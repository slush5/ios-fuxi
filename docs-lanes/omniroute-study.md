# OmniRoute study (agent quota stretching, ToS and security review)

Scope: read-only study of the code. Nothing was installed, run, built or
configured, and no account or credential was used. The repos were cloned into a
throwaway scratch dir only to read them.

Source tags (every code claim cites one of these plus `path:line`):

- **[F]** `ChrisCompton/omniroute` at commit `515674b6cf7c` (fork HEAD, the
  only branch, `main`).
- **[U]** `diegosouzapw/OmniRoute` at commit `b2a64106c9e4` (upstream
  `HEAD` = `release/v3.8.53`, 2026-10-30). `cheaper-inference/OmniRoute`
  advertises the very same `HEAD` sha (`git ls-remote`), so it is the same
  repo under a second name (**[I]** transfer or rename with a GitHub redirect).
- **[I]** marks an inference that the code does not show directly.
- **[T]** marks a statement taken from a provider's published terms or docs
  (URL given in section 2).

Short verdict up front: **avoid** the subscription routes entirely; OmniRoute
is useful to us, at most, as a local router for **official API keys** on free
or cheap tiers, and even that is optional because a few lines of config in our
own scripts do the same job with a far smaller attack surface. Details in
section 4.

## 1. What it actually does (from the code)

### 1.1 Architecture

| Piece | What it is | Source |
|---|---|---|
| Server | Next.js app plus an `open-sse` workspace that holds the router core | [F] `package.json:22-24` (workspaces), `package.json:51` (dev script) |
| Default bind | `HOSTNAME=0.0.0.0` on port 20128, so it listens on every interface | [F] `bin/omniroute.mjs:251`, `Dockerfile:27` |
| Provider table | One static registry object; each entry has `baseUrl`, `authType` (`oauth` or `apikey`), spoofed `headers`, and an `oauth` block that names an env var for the client id | [F] `open-sse/config/providerRegistry.ts:133` onward |
| Executors | Per-family request code (`antigravity`, `gemini-cli`, `codex`, `github`, `cursor`, `kiro`, `default`, ...) | [F] `open-sse/executors/*.ts` |
| Translators | OpenAI to Claude / Gemini / Responses format conversion | [F] `open-sse/translator/request/openai-to-claude.ts` |
| Fallback and pooling | "Combos" (ordered model lists), per-provider multi-account selection with `fill-first`, `round-robin`, `p2c`, `random` | [F] `open-sse/services/accountSelector.ts:19`, `:42-67`; `open-sse/services/combo.ts` |
| Storage | SQLite `storage.sqlite` in the data dir | [F] `src/lib/db/core.ts:25-29` |
| MITM proxy | Local TLS man-in-the-middle for `daily-cloudcode-pa.googleapis.com` (Antigravity backend), see 1.4 | [F] `src/mitm/` |
| Extras | MCP server (`open-sse/mcp-server`), Electron shell (`electron/`) | [F] `package.json:13` |

Adding a provider = adding an entry to `REGISTRY` in
`open-sse/config/providerRegistry.ts`, and, for subscription providers, an
OAuth service under `src/lib/oauth/services/` plus constants in
`src/lib/oauth/constants/oauth.ts` ([F]). OpenAI-compatible key providers need
only the registry entry and the `default` executor.

### 1.2 How it reaches "free" tiers

Two very different mechanisms live side by side:

1. **Official API keys** (`authType: "apikey"`): Gemini API
   (`generativelanguage.googleapis.com`), OpenRouter, Groq, Cerebras, Mistral,
   NVIDIA, OpenAI, Anthropic API, DeepSeek and about 30 more ([F]
   `open-sse/config/providerRegistry.ts:176-186` Gemini, `:497-503` OpenAI,
   `:518-525` Anthropic, `:585-591` OpenRouter, `:889-895` Groq, `:944-950`
   Mistral, `:1010-1016` Cerebras). Plain bearer or `x-api-key`. This is the
   legitimate part.
2. **Consumer subscription logins** (`authType: "oauth"` or token import):
   OmniRoute logs in **as the vendor's own first-party CLI or IDE**, using
   that client's OAuth client id (and for Google, its client secret), then
   sends requests with that client's User-Agent, beta flags, header order, and
   in Claude's case its system prompt. That is how "free" Claude, Gemini 3,
   GPT-5 etc. are reached: it rides the user's Pro/Max/AI Pro/ChatGPT/Copilot
   subscription quota through private backends.

The client ids are **not** OmniRoute's own registered apps. They are shipped as
defaults in `.env.example` and copied into `.env` on install by
`scripts/sync-env.mjs` ([F] `src/lib/oauth/constants/oauth.ts:1-8` says so;
`.env.example:111-135` holds them). **[I]** They match the public client ids
embedded in the vendors' own CLIs (Claude Code, Codex CLI, Gemini CLI,
Antigravity, VS Code Copilot). The upstream still ships the Claude one at
[U] `.env.example:1622`. Secrets are not reproduced here on purpose.

### 1.3 Subscription-reuse providers, with the auth mechanism

| Provider (subscription) | Mechanism | Source |
|---|---|---|
| **Claude Pro/Max** (`claude`, alias `cc`) | OAuth PKCE at `claude.ai/oauth/authorize` with the Claude Code client id, scope `user:inference` and `user:sessions:claude_code` | [F] `src/lib/oauth/constants/oauth.ts:11-25`, `.env.example:111` |
| | Requests to `api.anthropic.com/v1/messages?beta=true` with `User-Agent: claude-cli/2.1.63 (external, cli)`, `X-App: cli`, `claude-code-20250219,oauth-2025-04-20,...` betas and fake Stainless SDK headers | [F] `open-sse/config/providerRegistry.ts:135-160` |
| | Every translated request gets the system block "You are Claude Code, Anthropic's official CLI for Claude." prepended | [F] `open-sse/config/constants.ts:26`, `open-sse/translator/request/openai-to-claude.ts:314-320`; still present at [U] `open-sse/config/constants.ts:131`, `open-sse/executors/base.ts:1217` |
| | Header and body field order copied from the real CLI ("CLI fingerprint") | [F] `open-sse/config/cliFingerprints.ts:54` onward |
| | `anthropic-compatible-cc-*` variant adds a forged `x-anthropic-billing-header: cc_version=2.1.89...; cc_entrypoint=sdk-cli` block and a fake CWD/date Claude Code preamble | [F] `open-sse/services/claudeCodeCompatible.ts:10-14`, `:94-98`, `:400-408` |
| **Gemini CLI / Google AI Pro** (`gemini-cli`) | Google OAuth with the Gemini CLI client id and secret, scope `cloud-platform` | [F] `src/lib/oauth/constants/oauth.ts:43-55`, `.env.example:117-122` |
| | Calls the private Code Assist backend `cloudcode-pa.googleapis.com/v1internal` (`loadCodeAssist`, `generateContent`) as `User-Agent: GeminiCLI/0.31.0` | [F] `open-sse/config/providerRegistry.ts:200-229`, `open-sse/executors/gemini-cli.ts:4`, `:27-29`, `:73-80` |
| **Antigravity / Google AI Pro** (`antigravity`) | Google OAuth with the Antigravity client id and secret, extra internal scopes `cclog` and `experimentsandconfigs` | [F] `src/lib/oauth/constants/oauth.ts:116-137`, `.env.example:131-132` |
| | Calls `daily-cloudcode-pa.googleapis.com`, its `.sandbox` twin, and `cloudcode-pa`, `v1internal:streamGenerateContent`, as `User-Agent: antigravity/1.107.0 <os>/<arch>`; exposes the Antigravity pools' Claude Opus/Sonnet, Gemini 3.1 and GPT-OSS models | [F] `open-sse/config/providerRegistry.ts:345-382`, `open-sse/executors/antigravity.ts:38`, `:44-45` |
| **ChatGPT Plus/Pro** (`codex`) | OpenAI OAuth with the Codex CLI client id, `originator: codex_cli_rs` | [F] `src/lib/oauth/constants/oauth.ts:28-40`, `.env.example:114` |
| | Calls `chatgpt.com/backend-api/codex/responses` as `codex-cli/0.92.0 (Windows 10.0.26100; x64)` with a `chatgpt-account-id` header | [F] `open-sse/config/providerRegistry.ts:231-244`, `open-sse/executors/codex.ts:324`, `:360` |
| **GitHub Copilot** (`github`) | Device flow with the VS Code Copilot client id, then `api.github.com/copilot_internal/v2/token` | [F] `src/lib/oauth/constants/oauth.ts:153-164`, `.env.example:135`, `open-sse/executors/github.ts:154-158` |
| | Chat at `api.githubcopilot.com` posing as `GitHubCopilotChat/0.38.0`, `editor-version: vscode/1.110.0`, `copilot-integration-id: vscode-chat`; also the only executor that uses the TLS spoofing client | [F] `open-sse/config/providerRegistry.ts:384-398`, `open-sse/executors/github.ts` (imports `wreq-js`) |
| **Cursor** | Reads `cursorAuth/accessToken` and `storage.serviceMachineId` straight out of Cursor's local `state.vscdb` (auto-import) | [F] `src/app/api/oauth/cursor/auto-import/route.ts:26-30`, `src/lib/oauth/constants/oauth.ts:195-214` |
| **Kiro (AWS)** | AWS Builder ID device flow or Kiro social login, or paste a refresh token from the Kiro IDE; CodeWhisperer backend | [F] `src/lib/oauth/constants/oauth.ts:172-193`, `open-sse/config/providerRegistry.ts:435-441` |
| Qwen, Kimi Coding, KiloCode, Cline, Qoder | Same pattern with those vendors' CLI client ids | [F] `src/lib/oauth/constants/oauth.ts:57-113`, `.env.example:125-128`, `:138` |

### 1.4 Detection evasion is a stated feature

This is not incidental header copying. The README advertises it:

- "TLS Fingerprint Spoofing, Browser-like TLS fingerprint via `wreq-js` to
  bypass bot detection" ([F] `README.md:280`); code: `browser: "chrome_124"`
  ([F] `open-sse/utils/tlsClient.ts:86`), switched on in the chat path ([F]
  `open-sse/utils/proxyFetch.ts:248`, `src/sse/handlers/chatHelpers.ts:151`).
- "CLI Fingerprint Matching ... drastically reducing account flagging risk"
  and "reduces ban risk" ([F] `README.md:281`, `:1407`).
- "Auto-Disable Banned Accounts" circuit breaker ([F] `README.md:1413`),
  i.e. the design expects accounts to get banned and rotates to the next one.
- Multi-account rotation per provider ([F]
  `open-sse/services/accountSelector.ts:42-67`) lets one user pool several
  subscriptions behind one endpoint.
- **MITM of Antigravity**: with the user's sudo password it writes
  `127.0.0.1 daily-cloudcode-pa.googleapis.com` into `/etc/hosts` ([F]
  `src/mitm/dns/dnsConfig.ts:5`, `:69-77`), generates a cert and installs it
  as a **trusted root** in the system keychain / Windows Root store ([F]
  `src/mitm/cert/install.ts:72`, `:86`), and runs a local server on port 443
  that forwards the Antigravity IDE's own traffic to the OmniRoute router
  ([F] `src/mitm/server.cjs:16-21`, `:278`). Kiro reuses the same setup ([F]
  `src/mitm/targets/kiro.ts:49-53`).

## 2. Terms of service risk

Network note: this session's proxy blocks `antigravity.google`,
`ai.google.dev`, `developers.google.com` and `docs.github.com`, so the Google
and GitHub texts below come from search snippets of those pages and from
reporting, not a direct read. The Anthropic pages were read directly.

### 2.1 Anthropic (Claude Pro/Max/Ultra)

Read directly:

- Consumer Terms (https://www.anthropic.com/legal/consumer-terms), section 3
  item 7 [T]: you may not "access the Services through automated or non-human
  means, whether through a bot, script, or otherwise", except via an API key
  or where Anthropic explicitly permits it. Section 2: no sharing of account
  login or credentials. Section 12: Anthropic "may suspend or terminate your
  access to the Services (including any Subscriptions) at any time without
  notice", no refund on a terms breach.
- Claude Code legal page
  (https://code.claude.com/docs/en/legal-and-compliance), "Authentication and
  credential use" [T]: OAuth is "intended exclusively" for subscribers "to
  support ordinary use of Claude Code and other native Anthropic
  applications"; third parties may not "route requests through Free, Pro,
  Max, or Ultra plan credentials"; developers "may not collect, store, or
  intermediate Claude.ai credentials or session tokens"; and "Anthropic
  reserves the right to take measures to enforce these restrictions and may
  do so without prior notice."

OmniRoute's `claude` route does exactly what that page forbids: it collects
and stores the OAuth tokens (section 3.1) and intermediates every request,
while pretending to be the Claude Code binary. **Clear breach.** Enforcement
is real: server-side blocking of spoofed Claude Code clients from January
2026 and the explicit legal text from February 2026 were widely reported
(https://www.theregister.com/2026/02/20/anthropic_clarifies_ban_third_party_claude_access,
https://winbuzzer.com/2026/02/19/anthropic-bans-claude-subscription-oauth-in-third-party-apps-xcxwbn/).
Ban risk: **high**, and the account is the employer's.

### 2.2 Google (Google AI Pro, Gemini CLI, Antigravity)

- Gemini CLI FAQ (https://geminicli.com/docs/resources/faq/, mirror
  https://raw.githubusercontent.com/google-gemini/gemini-cli/main/docs/resources/faq.md)
  [T, via search snippet]: using third-party software "to harvest or
  piggyback on Gemini CLI's OAuth authentication to access our backend
  services is a direct violation of our applicable terms and policies."
- Gemini CLI terms page (https://www.geminicli.com/docs/resources/tos-privacy):
  the Code Assist backend is governed by Google's terms, not by the CLI's
  Apache 2.0 license ([T] per
  https://codenote.pages.dev/en/posts/gemini-cli-apache-license-tos-gap/).
- Antigravity terms: https://antigravity.google/terms (could not be fetched
  here; **[I]** same Google APIs ToS family,
  https://developers.google.com/terms, which forbids misrepresenting the
  identity of your API client).
- Enforcement: in February 2026 Google suspended many accounts (AI Ultra
  subscribers included) that reached Antigravity through OpenClaw/OpenCode
  OAuth plugins, with 403 "violation of Terms of Service" errors
  (https://www.theregister.com/2026/02/23/google_antigravity_compute_burden/,
  https://www.techzine.eu/news/privacy-compliance/139032/google-intervenes-in-heavy-use-of-antigravity-and-gemini,
  https://discuss.ai.google.dev/t/urgent-mass-403-tos-bans-on-gemini-api-antigravity-for-open-source-cli-users-paid-tier/124508).
  Some reports claim the whole Google account was hit; that is not
  confirmed by the established outlets.

OmniRoute's `gemini-cli` and `antigravity` routes are this exact pattern,
plus a root-CA MITM of the Antigravity backend. **Clear breach, ban risk
high.** **[I]** A ban would most likely also kill our working `agy` headless
setup on the same account, since it is the same Code Assist entitlement.

### 2.3 Others

- ChatGPT via Codex OAuth: OpenAI Terms of Use
  (https://openai.com/policies/terms-of-use) forbid circumventing rate limits
  and programmatic extraction outside the API **[I]** (not fetched). Same
  spoofing pattern, same risk class.
- GitHub Copilot: GitHub Terms for Additional Products, Copilot section
  (https://docs.github.com/en/site-policy/github-terms/github-terms-for-additional-products-and-features)
  **[I]** (not fetched) covers excessive automated bulk use and can suspend
  Copilot access. Using the internal token endpoint while posing as VS Code
  plus TLS spoofing is the same risk class.
- Cursor and Kiro token import: scraping another app's local token store;
  **[I]** same risk class.

Not relevant to us today, but worth saying: for every one of these, account
rotation (section 1.4) also stacks a "multiple accounts to evade limits"
violation on top.

## 3. Security

### 3.1 Credentials at rest

- Where: SQLite `storage.sqlite` (plus `db.json` legacy and `db_backups/`) in
  the data dir ([F] `src/lib/db/core.ts:25-29`). Data dir is `~/.omniroute`,
  `%APPDATA%\omniroute` or `$XDG_CONFIG_HOME/omniroute`, or `DATA_DIR` ([F]
  `src/lib/dataPaths.ts:4`, `:22`, `:29-36`).
- How: optional field-level AES-256-GCM over `apiKey`, `accessToken`,
  `refreshToken`, `idToken` ([F] `src/lib/db/encryption.ts:13`, `:122-125`).
  If `STORAGE_ENCRYPTION_KEY` is unset it is **plaintext passthrough** ([F]
  `src/lib/db/encryption.ts:7-8`, `:36-37`). The key is scrypt-derived with a
  **fixed salt** `omniroute-field-encryption-v1` ([F] `:39-41`).
- `sync-env.mjs` auto-generates `STORAGE_ENCRYPTION_KEY`, `JWT_SECRET`,
  `API_KEY_SECRET` into the `.env` file on install ([F]
  `scripts/sync-env.mjs:21-24`, `:98-99`), so the key sits in plaintext next to
  the app **[I]**: anyone who can read the user's files gets both DB and key,
  so the encryption mostly protects stolen DB copies only.
- Dashboard login defaults to `INITIAL_PASSWORD=CHANGEME` ([F]
  `.env.example:13`) and the server binds `0.0.0.0` ([F]
  `bin/omniroute.mjs:251`): on a shared LAN, a forgotten password change
  exposes every stored token. **[I]** For our shop PC this is a real risk.
- `keytar` is only an optional dependency ([F] `package.json:129`); OS
  keychain is not the default store.
- The MITM feature asks for the **sudo password** in the web UI and caches it
  in process memory ([F] `src/mitm/manager.ts:13-24`), and installs a trusted
  root CA ([F] `src/mitm/cert/install.ts:72`). A leaked CA key would allow
  interception of that host by anyone holding it **[I]**.

### 3.2 Telemetry and calls home

No analytics SDK (no PostHog, Sentry, Segment etc.) and no call to an
OmniRoute-owned host was found in `src`, `open-sse`, `bin`, `electron`,
`scripts` ([F], grep of all URL literals; `omniroute.online` only appears in
i18n docs text, `scripts/i18n/apply-priority-overrides.mjs:164`). Outbound
non-provider calls found:

| Call | Trigger | Source |
|---|---|---|
| `api.github.com/repos/router-for-me/CLIProxyAPI/releases` | version manager "check update" for a bundled third-party proxy binary | [F] `src/lib/versionManager/releaseChecker.ts:52-71` |
| `api.github.com/repos/<owner>/<repo>/issues` | user-initiated "report issue" | [F] `src/app/api/v1/issues/report/route.ts:94` |
| Electron `electron-updater` against GitHub releases of `diegosouzapw/OmniRoute`, `autoInstallOnAppQuit = true` | desktop app | [F] `electron/main.js:34`, `:145-146`, `electron/package.json:39-43` |

The Electron auto-updater means the desktop app will install whatever the
upstream maintainer publishes, unsigned-by-us, on quit **[I]**.

`node-machine-id` ([F] `package.json:111`) is used to embed a machine id into
locally issued router API keys (`sk-{machineId}-{keyId}-{crc8}`, [F]
`src/shared/utils/apiKey.ts:41`); no network use of it was found.

### 3.3 Fork versus upstream

- The fork's only branch `main` points at `515674b6`, which **is an upstream
  commit** ("chore(release): v3.6.1", 2026-04-10, by the upstream author). The
  fork has **zero commits of its own**; there is no diff to review.
- It is simply a stale snapshot: upstream is **10,206 commits ahead**
  (`git rev-list --count 515674b6..b2a64106`). Using the fork would mean
  running a six-month-old build with none of the later fixes.
- The mechanisms above are all still present upstream (Claude Code client id
  [U] `.env.example:1622`, spoofed system prompt [U]
  `open-sse/config/constants.ts:131`, `wreq-js` pinned at 3.2.0 [U]
  `package.json:441`, postinstall [U] `package.json:304`). The upstream
  repo is also huge (about 3.4 GB clone), which makes a full audit of each
  update impractical for us **[I]**.

### 3.4 Dependency red flags

- `postinstall` runs `scripts/postinstall.mjs` ([F] `package.json:87`). Read
  in full: it copies a `better-sqlite3` native binary into the standalone app
  and, on failure, runs `prebuild-install` / `npm rebuild` via `execSync` ([F]
  `scripts/postinstall.mjs:48-80`, `:106`, `:138`). Benign as written, but it
  executes native build tooling at install time.
- `wreq-js` ([F] `package.json:123`) is a native TLS-impersonation module
  **[I]** shipping prebuilt binaries; its only purpose here is to defeat bot
  detection. Red flag on purpose, not on code quality.
- No `eval`, `new Function`, long base64 blobs, or source files over 200 KB
  found in `src`, `open-sse`, `bin`, `scripts`, `electron` ([F], grep and find).
  `node_modules` was not installed, so transitive deps were not audited.
- Shipping other vendors' OAuth client secrets in `.env.example` ([F]
  `.env.example:117-132`) is itself a policy problem, independent of us.

## 4. Fit for us and recommendation

Our setup: Claude Code on the employer's Max/Ultra plan (weekly cap), and
Antigravity `agy` headless on Google AI Pro (separate Gemini, Claude+GPT and
Argon pools).

- **Stretching Claude or Antigravity quota through OmniRoute:** only possible
  through sections 1.3 and 1.4, which breach the published terms of both
  vendors (section 2), on accounts the employer owns, with documented ban
  waves in 2026. **Not acceptable.** It would also put the employer's OAuth
  tokens into a third-party SQLite file, plaintext by default, behind a
  `0.0.0.0` dashboard.
- **Routing cheap side tasks to official free API tiers** (citation checks,
  doc summaries): technically OmniRoute can do it with `authType: "apikey"`
  providers only (Gemini API key, OpenRouter `:free` models, Groq, Cerebras).
  That is legitimate under those providers' terms, with caveats:
  - Gemini API unpaid tier: Google may use prompts and outputs to improve
    products, and human reviewers may read them; "Do not submit sensitive,
    confidential, or personal information to the Unpaid Services"
    (https://ai.google.dev/gemini-api/terms, [T] via search snippet).
    Fine for public citations, **not** for our private repo content, vendor
    firmware notes, or anything from the employer.
  - OpenRouter free models (https://openrouter.ai/terms) and Groq
    (https://groq.com/terms-of-use): free routes may log or train depending
    on the upstream host **[I]**; same rule, public material only.
  - The keys must be personal or project keys created for this, never the
    employer's account login.
- But for that use OmniRoute is overkill: one OpenAI-compatible base URL and
  a key per provider in our own small script gives the same result without a
  Next.js server, a 0.0.0.0 dashboard, a root-CA MITM feature, a TLS spoofer,
  and an auto-updating Electron binary in the trust path **[I]**.

**Recommendation: avoid OmniRoute.** If the owner still wants a router, the
only acceptable configuration is: upstream (not this stale fork), CLI only (no
Electron), bound to `127.0.0.1`, `STORAGE_ENCRYPTION_KEY` set, password
changed, **only** `apikey` providers enabled with dedicated free-tier keys,
**no** OAuth provider ever connected, MITM never started, and only public,
non-sensitive text sent through it. A simpler and safer path for the same
goal is a direct call to the Gemini API free tier or OpenRouter `:free`
models from our own helper script, with the same "public material only" rule,
and Antigravity output still reviewed before use.

To read Google's and GitHub's terms directly next time, allow `antigravity.google`,
`ai.google.dev`, `developers.google.com` and `docs.github.com` in the cloud
session's network settings.
