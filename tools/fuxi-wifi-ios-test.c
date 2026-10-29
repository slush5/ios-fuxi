/* Root-only synthetic P-WIFI ioctl/BPF probe. Run only on an opted-in test boot.
 * Build on macOS: xcrun --sdk iphoneos clang -arch arm64 -miphoneos-version-min=16.7
 *   -O2 tools/fuxi-wifi-ios-test.c -o fuxi-wifi-ios-test
 * Sign/stage with the project's existing local userland tooling.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "../xnu/fuxi-wifi-ios/fuxi_wifi_apple80211.h"
#ifdef __APPLE__
#include <net/bpf.h>
#include <net/if.h>
#include <poll.h>
#endif

static int
request(int socket, bool set, int type, void *data, unsigned length, int *value)
{
	struct apple80211req req = { .req_if_name = "en0", .req_type = type,
		.req_len = length, .req_data = (uintptr_t)data };
	int rc = ioctl(socket, set ? SIOCSA80211 : SIOCGA80211, &req);
	if (value != NULL) {
		*value = req.req_val;
	}
	return rc;
}

#ifdef __APPLE__
static int
loopback(int socket)
{
	char path[32];
	int fd = -1;
	for (unsigned i = 0; i < 256; i++) {
		snprintf(path, sizeof(path), "/dev/bpf%u", i);
		fd = open(path, O_RDWR);
		if (fd >= 0 || errno != EBUSY) {
			break;
		}
	}
	if (fd < 0) {
		return -1;
	}
	struct ifreq interface = { .ifr_name = "en0" };
	unsigned yes = 1, no = 0, size;
	int rc = -1;
	uint8_t *buffer = NULL;
	if (ioctl(socket, SIOCGIFFLAGS, &interface) < 0) {
		goto out;
	}
	interface.ifr_flags |= IFF_UP;
	if (ioctl(socket, SIOCSIFFLAGS, &interface) < 0 || ioctl(fd, BIOCSETIF, &interface) < 0 ||
	    ioctl(fd, BIOCIMMEDIATE, &yes) < 0 || ioctl(fd, BIOCSSEESENT, &no) < 0 ||
	    ioctl(fd, BIOCSHDRCMPLT, &yes) < 0 || ioctl(fd, BIOCGBLEN, &size) < 0 || size > 1048576) {
		goto out;
	}
	buffer = malloc(size);
	if (buffer == NULL) {
		goto out;
	}
	uint8_t frame[60] = { 0x02, 0x46, 0x55, 0x58, 0x49, 0x01,
		0x02, 0x46, 0x55, 0x58, 0x49, 0x01, 0x88, 0xb5 };
	memcpy(frame + 14, "FUXI-P-WIFI-LOOPBACK", 20);
	if (write(fd, frame, sizeof(frame)) != sizeof(frame)) {
		goto out;
	}
	for (unsigned i = 0; i < 20; i++) {
		struct pollfd poller = { .fd = fd, .events = POLLIN };
		if (poll(&poller, 1, 100) <= 0) {
			continue;
		}
		ssize_t count = read(fd, buffer, size);
		for (size_t at = 0; count > 0 && at + sizeof(struct bpf_hdr) <= (size_t)count;) {
			struct bpf_hdr *header = (void *)(buffer + at);
			if (header->bh_hdrlen < sizeof(*header) || at + header->bh_hdrlen > (size_t)count ||
			    header->bh_caplen > (size_t)count - at - header->bh_hdrlen) {
				break;
			}
			if (header->bh_caplen == sizeof(frame) &&
			    memcmp(buffer + at + header->bh_hdrlen, frame, sizeof(frame)) == 0) {
				rc = 0;
				goto out;
			}
			at += BPF_WORDALIGN(header->bh_hdrlen + header->bh_caplen);
		}
	}
	errno = ETIMEDOUT;
out:
	free(buffer);
	close(fd);
	return rc;
}
#endif

int
main(int argc, char **argv)
{
	if (geteuid() != 0 || argc > 2 || (argc == 2 && strcmp(argv[1], "--loopback") != 0)) {
		fprintf(stderr, "usage (root, synthetic boot only): fuxi-wifi-ios-test [--loopback]\n");
		return 2;
	}
	int socket_fd = socket(AF_INET, SOCK_DGRAM, 0), rc = 1, value = 0;
	if (socket_fd < 0) {
		perror("socket");
		return 1;
	}
	char driver[256] = { 0 };
	if (request(socket_fd, false, 43, driver, sizeof(driver), NULL) < 0 ||
	    strcmp(driver, "Fuxi Wi-Fi iOS shim (synthetic backend)") != 0) {
		fprintf(stderr, "P-WIFI TEST: requires the synthetic Fuxi shim\n");
		close(socket_fd);
		return 2;
	}
	/* what wifid of 20H392 reads at start and the shim now answers (gets only: nothing is changed) */
	char country[3] = { 0 };
	struct apple80211_phy_mode phy = { 0 };
	struct apple80211_rssi_bounds bounds = { 0 };
	if (request(socket_fd, false, 51, country, sizeof(country), NULL) < 0 ||
	    request(socket_fd, false, 14, &phy, sizeof(phy), NULL) < 0 ||
	    request(socket_fd, false, 106, &bounds, sizeof(bounds), NULL) < 0) {
		goto out;
	}
	if (phy.version != 1 || bounds.version != 1 || country[2] != '\0') {
		errno = EPROTO;
		goto out;
	}
	printf("P-WIFI TEST: answers PASS (country %.2s, 802.11 modes %#x, RSSI bounds %d to %d dBm)\n", country,
	    (unsigned)phy.supported, (int)bounds.rssi_min, (int)bounds.rssi_max);
	struct { uint32_t version, radios, state[4]; } power = { 1, 1, { 1 } };
	struct apple80211_scan_request scan = { .version = 1 };
	struct apple80211_scan_result result = { .elements_length = 1024 };
	struct apple80211_assoc join = { .version = 1, .mode = 1, .auth_lower = 1, .ssid_length = 13 };
	if (request(socket_fd, true, 19, &power, sizeof(power), NULL) < 0 ||
	    request(socket_fd, true, 10, &scan, sizeof(scan), NULL) < 0) {
		goto out;
	}
	bool heard = false;
	for (unsigned i = 0; i < 100; i++) {
		if (request(socket_fd, false, 11, &result, sizeof(result), NULL) == 0 &&
		    result.ssid_length == 13 && memcmp(result.ssid, "Fuxi Loopback", 13) == 0) {
			heard = true;
			break;
		}
		usleep(50000);
	}
	if (!heard) {
		errno = ETIMEDOUT;
		goto out;
	}
	puts("P-WIFI TEST: scan PASS (synthetic BSS)");
	memcpy(join.ssid, "Fuxi Loopback", 13);
	if (request(socket_fd, true, 20, &join, sizeof(join), NULL) < 0) {
		goto out;
	}
	for (unsigned i = 0; i < 100; i++) {
		if (request(socket_fd, false, 21, NULL, 0, &value) == 0 && value != 0) {
			break;
		}
		usleep(50000);
	}
	if (value != 1) {
		errno = ETIMEDOUT;
		goto out;
	}
	puts("P-WIFI TEST: connect PASS (no radio/DHCP)");
	if (argc == 2) {
#ifdef __APPLE__
		if (loopback(socket_fd) < 0) {
			goto out;
		}
		puts("P-WIFI TEST: RX loopback PASS (outgoing BPF capture disabled)");
#else
		errno = ENOTSUP;
		goto out;
#endif
	}
	if (request(socket_fd, true, 22, NULL, 0, NULL) < 0) {
		goto out;
	}
	puts("P-WIFI TEST: disconnect queued; PASS");
	rc = 0;
out:
	if (rc) {
		perror("P-WIFI TEST: FAIL");
	}
	close(socket_fd);
	return rc;
}
