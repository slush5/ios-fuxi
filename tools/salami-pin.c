/*
 * salami-pin: reserve, pin and map the iOS root file on OnePlus 11 (salami) userdata.
 *
 * Runs on the phone as root (Magisk su). Stock OxygenOS ships no f2fs_io, so this
 * static helper does the f2fs ioctls itself. Build (static aarch64, no libc on the
 * phone needed):
 *   source ~/op11-ios/toolchain/env.sh
 *   $ZIG cc -target aarch64-linux-musl -static -O2 -s -Wall -Wextra \
 *       -o tools/out/salami-pin tools/salami-pin.c
 *
 *   salami-pin create PATH SIZE [opts]         create PATH (must not exist), pin it,
 *                                              fallocate SIZE bytes, fsync, verify,
 *                                              print the JSON report
 *   salami-pin verify PATH SIZE [opts]         open PATH read-only, verify, print JSON
 *   salami-pin check PATH SIZE DIGEST [opts]   verify silently; exit 0 only if the
 *                                              extent digest equals DIGEST
 * opts: --part NAME      partition under the userdata dm device (default sda14)
 *       --max-extents N  refuse more merged extents (default 512 = FX_MAX_EXTENTS,
 *                        xnu/fuxi-unixnu/sweet_ufs.c; the 16 KiB ADT holds ~200)
 *       --allow-fbe      accept an FBE-encrypted file (not expected under /data/)
 *       --keep-on-fail   create: keep the file when a later step fails
 *
 * Why the file is never written: salami userdata is f2fs on dm-default-key
 * (metadata encryption, aes-xts, wrappedkey_v0), ONE target over sda14 at offset 0
 * with 4096-byte sectors. Every block written through /data reaches UFS as
 * ciphertext, while XNU (patch 0088 extent root) issues plain READ(10)/WRITE(10).
 * The file only reserves pinned blocks; tools/salami-install-root.sh writes the
 * image raw into /dev/block/sda14 at the extents printed here. Because the dm
 * target is 1:1, FIEMAP physical offsets (bytes on the dm device) are sda14 offsets.
 *
 * Verification (create and verify): regular file of exactly SIZE bytes, one link,
 * on f2fs with 4096-byte blocks; the file's device is the dm device named
 * "userdata" whose only slave is the partition and whose size equals it; pinned
 * (FS_NOCOW_FL, which f2fs reports for FI_PIN_FILE); no compression flag and
 * F2FS_IOC_GET_COMPRESS_BLOCKS refusing (file not compressed); not encrypted by
 * FBE; FIEMAP over [0, SIZE) is complete, contiguous from 0, 4 KiB aligned,
 * nonzero physical, only LAST/MERGED flags, final extent LAST, inside the
 * partition, no physical overlap.
 *
 * JSON (stdout, only on success): file, size, fs_dev, dm, dm_name, partition,
 * partition_bytes, pinned, gc_failures, gc_pin_file_thresh, fsflags, compressed,
 * extent_count, digest, extents (merged, clamped to SIZE: [logical, physical,
 * length] in bytes, the tools/mkadt-fuxi.py root_extent_props format) and fiemap
 * (raw kernel extents: logical, physical, length, flags).
 * digest = FNV-1a 64 over little-endian u64 SIZE then each merged triple.
 *
 * Exit status: 0 ok, 1 usage, 2 refused (state unsafe for the raw root),
 * 3 system call failed.
 */
#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <unistd.h>

/* include/uapi/linux/f2fs.h */
#define F2FS_IOCTL_MAGIC 0xf5
#define F2FS_IOC_SET_PIN_FILE _IOW(F2FS_IOCTL_MAGIC, 13, uint32_t)
#define F2FS_IOC_GET_PIN_FILE _IOR(F2FS_IOCTL_MAGIC, 14, uint32_t)
#define F2FS_IOC_GET_COMPRESS_BLOCKS _IOR(F2FS_IOCTL_MAGIC, 17, uint64_t)
#define F2FS_SUPER_MAGIC 0xF2F52010u /* include/uapi/linux/magic.h */

/* include/uapi/linux/fiemap.h (own names: no kernel headers needed) */
struct sp_fiemap_extent {
	uint64_t fe_logical;
	uint64_t fe_physical;
	uint64_t fe_length;
	uint64_t fe_reserved64[2];
	uint32_t fe_flags;
	uint32_t fe_reserved[3];
};
struct sp_fiemap {
	uint64_t fm_start;
	uint64_t fm_length;
	uint32_t fm_flags;
	uint32_t fm_mapped_extents;
	uint32_t fm_extent_count;
	uint32_t fm_reserved;
	struct sp_fiemap_extent fm_extents[];
};
#define FIEMAP_FLAG_SYNC 0x00000001u
#define FIEMAP_EXTENT_LAST 0x00000001u
#define FIEMAP_EXTENT_UNKNOWN 0x00000002u
#define FIEMAP_EXTENT_DELALLOC 0x00000004u
#define FIEMAP_EXTENT_ENCODED 0x00000008u
#define FIEMAP_EXTENT_DATA_ENCRYPTED 0x00000080u
#define FIEMAP_EXTENT_NOT_ALIGNED 0x00000100u
#define FIEMAP_EXTENT_DATA_INLINE 0x00000200u
#define FIEMAP_EXTENT_DATA_TAIL 0x00000400u
#define FIEMAP_EXTENT_UNWRITTEN 0x00000800u
#define FIEMAP_EXTENT_MERGED 0x00001000u
#define FIEMAP_EXTENT_SHARED 0x00002000u

/* include/uapi/linux/fs.h */
#define SP_FS_IOC_GETFLAGS _IOR('f', 1, long)
#define SP_FS_IOC_FIEMAP _IOWR('f', 11, struct sp_fiemap)
#define SP_FS_COMPR_FL 0x00000004u
#define SP_FS_ENCRYPT_FL 0x00000800u
#define SP_FS_NOCOW_FL 0x00800000u

_Static_assert(sizeof(struct sp_fiemap) == 32, "struct fiemap layout");
_Static_assert(sizeof(struct sp_fiemap_extent) == 56, "struct fiemap_extent layout");
_Static_assert(F2FS_IOC_SET_PIN_FILE == 0x4004f50d, "F2FS_IOC_SET_PIN_FILE");
_Static_assert(F2FS_IOC_GET_PIN_FILE == 0x8004f50e, "F2FS_IOC_GET_PIN_FILE");
_Static_assert(F2FS_IOC_GET_COMPRESS_BLOCKS == 0x8008f511, "F2FS_IOC_GET_COMPRESS_BLOCKS");
_Static_assert(SP_FS_IOC_GETFLAGS == 0x80086601, "FS_IOC_GETFLAGS (LP64)");
_Static_assert(SP_FS_IOC_FIEMAP == 0xc020660b, "FS_IOC_FIEMAP");

#define BLK 4096u
#define EXIT_USAGE 1
#define EXIT_REFUSED 2
#define EXIT_SYSCALL 3
#define MAX_SIZE (1ull << 40) /* sanity only; the partition bound is checked separately */

struct opts {
	const char *mode, *path, *part;
	uint64_t size, digest;
	unsigned max_extents;
	int allow_fbe, keep_on_fail, quiet;
};

struct report {
	unsigned dev_major, dev_minor;
	char dm[64], dm_name[64], partition[64];
	uint64_t partition_bytes;
	unsigned fsflags;
	uint32_t gc_failures;
	long gc_pin_file_thresh;
	struct sp_fiemap *fm;     /* raw kernel extents */
	uint64_t (*ext)[3];       /* merged, clamped triples */
	unsigned n_ext;
	uint64_t digest;
};

static int refuse(const char *fmt, ...)
{
	va_list ap;
	fputs("salami-pin: refused: ", stderr);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
	return EXIT_REFUSED;
}

static int sysfail(const char *what)
{
	fprintf(stderr, "salami-pin: %s: %s\n", what, strerror(errno));
	return EXIT_SYSCALL;
}

static int usage(void)
{
	fputs("usage: salami-pin create|verify PATH SIZE [opts]\n"
	      "       salami-pin check PATH SIZE DIGEST [opts]\n"
	      "opts: --part NAME (sda14) --max-extents N (512) --allow-fbe --keep-on-fail\n",
	      stderr);
	return EXIT_USAGE;
}

static int parse_u64(const char *s, uint64_t *out, int base)
{
	char *end;
	unsigned long long v;
	if (!s || !*s || *s == '-')
		return -1;
	errno = 0;
	v = strtoull(s, &end, base);
	if (errno || *end)
		return -1;
	*out = v;
	return 0;
}

/* Only plain path characters: the path goes into JSON and phone shell scripts. */
static int path_ok(const char *p)
{
	if (strncmp(p, "/data/", 6) != 0 || strlen(p) > 200 || strstr(p, "/.") || strstr(p, "//"))
		return 0;
	for (const char *c = p; *c; c++)
		if (!((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') ||
		      *c == '/' || *c == '.' || *c == '_' || *c == '-'))
			return 0;
	return p[strlen(p) - 1] != '/';
}

static int name_ok(const char *p)
{
	if (!*p || strlen(p) >= 64)
		return 0;
	for (const char *c = p; *c; c++)
		if (!((*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9')))
			return 0;
	return 1;
}

static int read_line(const char *path, char *buf, size_t n)
{
	FILE *f = fopen(path, "re");
	if (!f)
		return -1;
	if (!fgets(buf, (int)n, f)) {
		fclose(f);
		return -1;
	}
	fclose(f);
	buf[strcspn(buf, "\r\n")] = 0;
	return 0;
}

static int read_u64_file(const char *path, uint64_t *v)
{
	char buf[64];
	if (read_line(path, buf, sizeof buf) < 0)
		return -1;
	return parse_u64(buf, v, 10);
}

/*
 * The file's st_dev must be the dm device "userdata" with exactly one slave,
 * the expected partition, of the same size (with the dmctl offset-0 check done by
 * the PC scripts, FIEMAP physical == partition offset).
 */
static int check_device(dev_t dev, const struct opts *o, struct report *r)
{
	char p[256], link[256];
	uint64_t dm_sectors, part_sectors;
	ssize_t len;
	DIR *d;
	struct dirent *e;
	int slaves = 0;

	r->dev_major = major(dev);
	r->dev_minor = minor(dev);
	snprintf(p, sizeof p, "/sys/dev/block/%u:%u", r->dev_major, r->dev_minor);
	len = readlink(p, link, sizeof link - 1);
	if (len <= 0)
		return refuse("%s: file is not on a block device (%s)", p, strerror(errno));
	link[len] = 0;
	snprintf(r->dm, sizeof r->dm, "%s", strrchr(link, '/') ? strrchr(link, '/') + 1 : link);
	snprintf(p, sizeof p, "/sys/dev/block/%u:%u/dm/name", r->dev_major, r->dev_minor);
	if (read_line(p, r->dm_name, sizeof r->dm_name) < 0)
		return refuse("file is on %u:%u (%s), not a device-mapper device", r->dev_major,
			      r->dev_minor, r->dm);
	if (strcmp(r->dm_name, "userdata") != 0)
		return refuse("file is on dm device '%s', not 'userdata'", r->dm_name);
	snprintf(p, sizeof p, "/sys/dev/block/%u:%u/slaves", r->dev_major, r->dev_minor);
	d = opendir(p);
	if (!d)
		return sysfail(p);
	while ((e = readdir(d))) {
		if (e->d_name[0] == '.')
			continue;
		slaves++;
		snprintf(r->partition, sizeof r->partition, "%s", e->d_name);
	}
	closedir(d);
	if (slaves != 1 || strcmp(r->partition, o->part) != 0)
		return refuse("userdata dm device has %d slaves (last '%s'); expected only %s", slaves,
			      r->partition, o->part);
	snprintf(p, sizeof p, "/sys/class/block/%s/partition", o->part);
	if (access(p, F_OK) != 0)
		return refuse("%s is not a partition", o->part);
	snprintf(p, sizeof p, "/sys/dev/block/%u:%u/size", r->dev_major, r->dev_minor);
	if (read_u64_file(p, &dm_sectors) < 0)
		return sysfail(p);
	snprintf(p, sizeof p, "/sys/class/block/%s/size", o->part);
	if (read_u64_file(p, &part_sectors) < 0)
		return sysfail(p);
	if (dm_sectors != part_sectors || part_sectors == 0 || part_sectors % (BLK / 512))
		return refuse("userdata dm size %llu sectors != %s %llu sectors (not 1:1)",
			      (unsigned long long)dm_sectors, o->part, (unsigned long long)part_sectors);
	r->partition_bytes = part_sectors * 512;
	snprintf(p, sizeof p, "/sys/fs/f2fs/%s/gc_pin_file_thresh", r->dm);
	{
		uint64_t t;
		r->gc_pin_file_thresh = read_u64_file(p, &t) == 0 ? (long)t : -1;
	}
	return 0;
}

static int get_fsflags(int fd, unsigned *flags)
{
	long v = 0; /* kernel stores an int; the long buffer matches the ioctl size */
	if (ioctl(fd, SP_FS_IOC_GETFLAGS, &v) < 0)
		return -1;
	*flags = (unsigned)v;
	return 0;
}

static const char *flag_names(uint32_t f, char *buf, size_t n)
{
	static const struct { uint32_t bit; const char *name; } t[] = {
		{FIEMAP_EXTENT_LAST, "LAST"}, {FIEMAP_EXTENT_UNKNOWN, "UNKNOWN"},
		{FIEMAP_EXTENT_DELALLOC, "DELALLOC"}, {FIEMAP_EXTENT_ENCODED, "ENCODED"},
		{FIEMAP_EXTENT_DATA_ENCRYPTED, "DATA_ENCRYPTED"},
		{FIEMAP_EXTENT_NOT_ALIGNED, "NOT_ALIGNED"}, {FIEMAP_EXTENT_DATA_INLINE, "DATA_INLINE"},
		{FIEMAP_EXTENT_DATA_TAIL, "DATA_TAIL"}, {FIEMAP_EXTENT_UNWRITTEN, "UNWRITTEN"},
		{FIEMAP_EXTENT_MERGED, "MERGED"}, {FIEMAP_EXTENT_SHARED, "SHARED"},
	};
	buf[0] = 0;
	for (size_t i = 0; i < sizeof t / sizeof t[0]; i++)
		if (f & t[i].bit) {
			strncat(buf, buf[0] ? "|" : "", n - strlen(buf) - 1);
			strncat(buf, t[i].name, n - strlen(buf) - 1);
		}
	return buf[0] ? buf : "none";
}

/*
 * One complete FIEMAP over [0, size): count first (fm_extent_count 0), then map
 * with one spare slot. fm_mapped_extents below the capacity proves nothing was cut
 * off; if the file changed in between, retry.
 */
static int map_extents(int fd, uint64_t size, struct report *r)
{
	for (int attempt = 0; attempt < 3; attempt++) {
		struct sp_fiemap probe;
		uint32_t cap;
		memset(&probe, 0, sizeof probe);
		probe.fm_start = 0;
		probe.fm_length = size;
		probe.fm_flags = FIEMAP_FLAG_SYNC;
		if (ioctl(fd, SP_FS_IOC_FIEMAP, &probe) < 0)
			return sysfail("FS_IOC_FIEMAP (count)");
		if (probe.fm_mapped_extents == 0)
			return refuse("FIEMAP reports no extents (file not allocated)");
		if (probe.fm_mapped_extents > 1000000)
			return refuse("FIEMAP reports %u extents", probe.fm_mapped_extents);
		cap = probe.fm_mapped_extents + 1;
		free(r->fm);
		r->fm = calloc(1, sizeof(struct sp_fiemap) + (size_t)cap * sizeof(struct sp_fiemap_extent));
		if (!r->fm)
			return sysfail("calloc");
		r->fm->fm_start = 0;
		r->fm->fm_length = size;
		r->fm->fm_flags = FIEMAP_FLAG_SYNC;
		r->fm->fm_extent_count = cap;
		if (ioctl(fd, SP_FS_IOC_FIEMAP, r->fm) < 0)
			return sysfail("FS_IOC_FIEMAP");
		if (r->fm->fm_mapped_extents < cap)
			return 0;
	}
	return refuse("extent map kept changing while it was read");
}

static uint64_t fnv_u64(uint64_t h, uint64_t v)
{
	for (int i = 0; i < 8; i++) {
		h ^= (v >> (8 * i)) & 0xff;
		h *= 0x100000001b3ull;
	}
	return h;
}

static int check_extents(const struct opts *o, struct report *r)
{
	struct sp_fiemap *fm = r->fm;
	uint32_t allowed = FIEMAP_EXTENT_LAST | FIEMAP_EXTENT_MERGED |
			   (o->allow_fbe ? FIEMAP_EXTENT_DATA_ENCRYPTED : 0);
	uint64_t pos = 0;
	int saw_last = 0;
	char names[160];

	r->ext = calloc(fm->fm_mapped_extents, sizeof *r->ext);
	if (!r->ext)
		return sysfail("calloc");
	r->n_ext = 0;
	for (uint32_t i = 0; i < fm->fm_mapped_extents; i++) {
		const struct sp_fiemap_extent *e = &fm->fm_extents[i];
		uint64_t len;
		if (saw_last)
			return refuse("extent %u follows the extent marked LAST", i);
		if (e->fe_flags & ~allowed)
			return refuse("extent %u has flags 0x%x (%s)", i, e->fe_flags,
				      flag_names(e->fe_flags, names, sizeof names));
		if (e->fe_logical != pos)
			return refuse("extent %u starts at %llu, expected %llu (hole or overlap)", i,
				      (unsigned long long)e->fe_logical, (unsigned long long)pos);
		if (pos >= o->size)
			return refuse("extent %u maps data at %llu, past the %llu-byte root", i,
				      (unsigned long long)pos, (unsigned long long)o->size);
		if (!e->fe_length || e->fe_length % BLK || e->fe_physical % BLK || !e->fe_physical)
			return refuse("extent %u is not a 4 KiB aligned mapped extent (physical %llu length %llu)",
				      i, (unsigned long long)e->fe_physical, (unsigned long long)e->fe_length);
		if (e->fe_physical > r->partition_bytes ||
		    e->fe_length > r->partition_bytes - e->fe_physical)
			return refuse("extent %u (physical %llu length %llu) is outside %s (%llu bytes)", i,
				      (unsigned long long)e->fe_physical, (unsigned long long)e->fe_length,
				      r->partition, (unsigned long long)r->partition_bytes);
		len = e->fe_length < o->size - pos ? e->fe_length : o->size - pos;
		if (r->n_ext && r->ext[r->n_ext - 1][1] + r->ext[r->n_ext - 1][2] == e->fe_physical) {
			r->ext[r->n_ext - 1][2] += len;
		} else {
			r->ext[r->n_ext][0] = pos;
			r->ext[r->n_ext][1] = e->fe_physical;
			r->ext[r->n_ext][2] = len;
			r->n_ext++;
		}
		pos += e->fe_length;
		saw_last = !!(e->fe_flags & FIEMAP_EXTENT_LAST);
	}
	if (!saw_last)
		return refuse("the final extent is not marked FIEMAP_EXTENT_LAST");
	if (pos < o->size)
		return refuse("extents cover %llu of %llu bytes", (unsigned long long)pos,
			      (unsigned long long)o->size);
	if (r->n_ext > o->max_extents)
		return refuse("%u extents > limit %u (free space and recreate the file, or shrink the root)",
			      r->n_ext, o->max_extents);
	for (unsigned i = 0; i < r->n_ext; i++)
		for (unsigned j = i + 1; j < r->n_ext; j++)
			if (r->ext[i][1] < r->ext[j][1] + r->ext[j][2] &&
			    r->ext[j][1] < r->ext[i][1] + r->ext[i][2])
				return refuse("extents %u and %u overlap on %s", i, j, r->partition);
	r->digest = fnv_u64(0xcbf29ce484222325ull, o->size);
	for (unsigned i = 0; i < r->n_ext; i++)
		for (int k = 0; k < 3; k++)
			r->digest = fnv_u64(r->digest, r->ext[i][k]);
	return 0;
}

static int verify_fd(int fd, const struct opts *o, struct report *r)
{
	struct stat st;
	struct statfs sf;
	uint64_t cblocks = 0;
	int rc;

	if (fstat(fd, &st) < 0)
		return sysfail("fstat");
	if (!S_ISREG(st.st_mode))
		return refuse("%s is not a regular file", o->path);
	if (st.st_nlink != 1)
		return refuse("%s has %lu links", o->path, (unsigned long)st.st_nlink);
	if ((uint64_t)st.st_size != o->size)
		return refuse("%s is %llu bytes, expected %llu", o->path, (unsigned long long)st.st_size,
			      (unsigned long long)o->size);
	if ((uint64_t)st.st_blocks * 512 < o->size)
		return refuse("%s is not fully allocated (%llu of %llu bytes)", o->path,
			      (unsigned long long)st.st_blocks * 512, (unsigned long long)o->size);
	if (fstatfs(fd, &sf) < 0)
		return sysfail("fstatfs");
	if ((uint32_t)sf.f_type != F2FS_SUPER_MAGIC)
		return refuse("%s is not on f2fs (f_type 0x%lx)", o->path, (unsigned long)sf.f_type);
	if (sf.f_bsize != BLK)
		return refuse("f2fs block size %lu, expected %u", (unsigned long)sf.f_bsize, BLK);
	if ((rc = check_device(st.st_dev, o, r)))
		return rc;
	if (get_fsflags(fd, &r->fsflags) < 0)
		return sysfail("FS_IOC_GETFLAGS");
	if (!(r->fsflags & SP_FS_NOCOW_FL))
		return refuse("%s is not pinned (fsflags 0x%08x): Android GC may move it", o->path, r->fsflags);
	if (r->fsflags & SP_FS_COMPR_FL)
		return refuse("%s has the f2fs compression flag (fsflags 0x%08x)", o->path, r->fsflags);
	if ((r->fsflags & SP_FS_ENCRYPT_FL) && !o->allow_fbe)
		return refuse("%s is FBE-encrypted (create it directly under /data/)", o->path);
	if (ioctl(fd, F2FS_IOC_GET_PIN_FILE, &r->gc_failures) < 0)
		return sysfail("F2FS_IOC_GET_PIN_FILE");
	if (ioctl(fd, F2FS_IOC_GET_COMPRESS_BLOCKS, &cblocks) == 0)
		return refuse("%s is an f2fs compressed file (%llu compressed blocks)", o->path,
			      (unsigned long long)cblocks);
	if (errno != EINVAL && errno != EOPNOTSUPP && errno != ENOTTY)
		return sysfail("F2FS_IOC_GET_COMPRESS_BLOCKS");
	if (r->gc_pin_file_thresh > 0 && r->gc_failures >= (uint64_t)r->gc_pin_file_thresh / 2)
		fprintf(stderr, "salami-pin: warning: %u GC failures on the pin (threshold %ld)\n",
			r->gc_failures, r->gc_pin_file_thresh);
	if ((rc = map_extents(fd, o->size, r)))
		return rc;
	return check_extents(o, r);
}

static void print_json(const struct opts *o, const struct report *r)
{
	char names[160];
	printf("{\n  \"tool\": \"salami-pin\",\n  \"version\": 1,\n  \"mode\": \"%s\",\n", o->mode);
	printf("  \"file\": \"%s\",\n  \"size\": %llu,\n  \"block_size\": %u,\n", o->path,
	       (unsigned long long)o->size, BLK);
	printf("  \"fs_dev\": \"%u:%u\",\n  \"dm\": \"%s\",\n  \"dm_name\": \"%s\",\n", r->dev_major,
	       r->dev_minor, r->dm, r->dm_name);
	printf("  \"partition\": \"%s\",\n  \"partition_bytes\": %llu,\n", r->partition,
	       (unsigned long long)r->partition_bytes);
	printf("  \"pinned\": true,\n  \"gc_failures\": %u,\n  \"gc_pin_file_thresh\": %ld,\n",
	       r->gc_failures, r->gc_pin_file_thresh);
	printf("  \"fsflags\": \"0x%08x\",\n  \"compressed\": false,\n", r->fsflags);
	printf("  \"extent_count\": %u,\n  \"digest\": \"%016llx\",\n  \"extents\": [", r->n_ext,
	       (unsigned long long)r->digest);
	for (unsigned i = 0; i < r->n_ext; i++)
		printf("%s\n    [%llu, %llu, %llu]", i ? "," : "", (unsigned long long)r->ext[i][0],
		       (unsigned long long)r->ext[i][1], (unsigned long long)r->ext[i][2]);
	printf("\n  ],\n  \"fiemap\": [");
	for (uint32_t i = 0; i < r->fm->fm_mapped_extents; i++) {
		const struct sp_fiemap_extent *e = &r->fm->fm_extents[i];
		printf("%s\n    {\"logical\": %llu, \"physical\": %llu, \"length\": %llu, "
		       "\"flags\": \"0x%08x\", \"flag_names\": \"%s\"}",
		       i ? "," : "", (unsigned long long)e->fe_logical, (unsigned long long)e->fe_physical,
		       (unsigned long long)e->fe_length, e->fe_flags,
		       flag_names(e->fe_flags, names, sizeof names));
	}
	printf("\n  ]\n}\n");
	fflush(stdout);
}

static int do_create(const struct opts *o, struct report *r, int *created_fd)
{
	unsigned flags = 0;
	uint32_t pin = 1;
	int fd, rc;

	if (geteuid() != 0)
		return refuse("create needs root (run through Magisk su)");
	fd = open(o->path, O_RDWR | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0600);
	if (fd < 0)
		return errno == EEXIST ? refuse("%s already exists (use verify, or remove it first)", o->path)
				       : sysfail(o->path);
	*created_fd = fd;
	{
		struct statfs sf;
		struct stat st;
		if (fstatfs(fd, &sf) < 0)
			return sysfail("fstatfs");
		if ((uint32_t)sf.f_type != F2FS_SUPER_MAGIC)
			return refuse("%s is not on f2fs", o->path);
		if (fstat(fd, &st) < 0)
			return sysfail("fstat");
		if ((rc = check_device(st.st_dev, o, r)))
			return rc;
		if (o->size > r->partition_bytes)
			return refuse("size %llu exceeds %s", (unsigned long long)o->size, r->partition);
	}
	if (get_fsflags(fd, &flags) < 0)
		return sysfail("FS_IOC_GETFLAGS");
	if ((flags & SP_FS_ENCRYPT_FL) && !o->allow_fbe)
		return refuse("%s is FBE-encrypted (create it directly under /data/)", o->path);
	if (flags & SP_FS_COMPR_FL)
		fprintf(stderr, "salami-pin: new file inherited the compression flag; pinning clears it\n");
	/* Pin before allocating: f2fs then gives the file whole pinned sections. */
	if (ioctl(fd, F2FS_IOC_SET_PIN_FILE, &pin) < 0) {
		if (errno == EOPNOTSUPP)
			return refuse("F2FS_IOC_SET_PIN_FILE: compression could not be disabled");
		if (errno == EAGAIN)
			return refuse("F2FS_IOC_SET_PIN_FILE: too many GC failures, try again later");
		if (errno == EINVAL)
			return refuse("F2FS_IOC_SET_PIN_FILE: f2fs refuses pinning here (lfs mode?)");
		return sysfail("F2FS_IOC_SET_PIN_FILE");
	}
	if (fallocate(fd, 0, 0, (off_t)o->size) < 0)
		return sysfail("fallocate");
	if (fsync(fd) < 0)
		return sysfail("fsync");
	return verify_fd(fd, o, r);
}

int main(int argc, char **argv)
{
	struct opts o = {.part = "sda14", .max_extents = 512};
	struct report r;
	int fd = -1, created_fd = -1, rc, argi;

	memset(&r, 0, sizeof r);
	r.gc_pin_file_thresh = -1;
	if (argc < 4)
		return usage();
	o.mode = argv[1];
	o.path = argv[2];
	if (strcmp(o.mode, "create") && strcmp(o.mode, "verify") && strcmp(o.mode, "check"))
		return usage();
	if (parse_u64(argv[3], &o.size, 10) < 0)
		return usage();
	argi = 4;
	if (!strcmp(o.mode, "check")) {
		if (argc < 5 || strlen(argv[4]) != 16 || parse_u64(argv[4], &o.digest, 16) < 0)
			return usage();
		o.quiet = 1;
		argi = 5;
	}
	for (; argi < argc; argi++) {
		uint64_t v;
		if (!strcmp(argv[argi], "--part") && argi + 1 < argc) {
			o.part = argv[++argi];
		} else if (!strcmp(argv[argi], "--max-extents") && argi + 1 < argc &&
			   parse_u64(argv[argi + 1], &v, 10) == 0 && v > 0 && v <= 100000) {
			o.max_extents = (unsigned)v;
			argi++;
		} else if (!strcmp(argv[argi], "--allow-fbe")) {
			o.allow_fbe = 1;
		} else if (!strcmp(argv[argi], "--keep-on-fail")) {
			o.keep_on_fail = 1;
		} else {
			return usage();
		}
	}
	if (!path_ok(o.path))
		return refuse("path must be a plain absolute path under /data/ ([A-Za-z0-9._-/])");
	if (!name_ok(o.part))
		return refuse("bad partition name");
	if (!o.size || o.size % BLK || o.size > MAX_SIZE)
		return refuse("size must be a nonzero multiple of 4096 (got %llu)", (unsigned long long)o.size);

	if (!strcmp(o.mode, "create")) {
		rc = do_create(&o, &r, &created_fd);
		if (rc && created_fd >= 0 && !o.keep_on_fail) {
			/* Remove only the file this run created (same inode as our fd). */
			struct stat a, b;
			if (fstat(created_fd, &a) == 0 && lstat(o.path, &b) == 0 && a.st_dev == b.st_dev &&
			    a.st_ino == b.st_ino && unlink(o.path) == 0)
				fprintf(stderr, "salami-pin: removed the partially prepared %s\n", o.path);
			else
				fprintf(stderr, "salami-pin: could not remove %s, check it by hand\n", o.path);
		}
		if (created_fd >= 0)
			close(created_fd);
	} else {
		fd = open(o.path, O_RDONLY | O_NOFOLLOW | O_CLOEXEC);
		if (fd < 0)
			return sysfail(o.path);
		rc = verify_fd(fd, &o, &r);
		close(fd);
		if (!rc && o.quiet && r.digest != o.digest)
			rc = refuse("extent digest %016llx differs from the expected %016llx",
				    (unsigned long long)r.digest, (unsigned long long)o.digest);
	}
	if (!rc && !o.quiet)
		print_json(&o, &r);
	free(r.fm);
	free(r.ext);
	return rc;
}
