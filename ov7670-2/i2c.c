#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

#define i2c_addr 0x21 // Given as 7-bit addr, 0x42/0x43 on bus

static const struct {
  int addr;
  int value;
} writelist [] = {
  { 0x3d, 0x81 }, // COM13, swap UV, turn off reserved bit 3
  { 0xb0, 0x84 },
  { 0x6f, 0x9f }, // AWBCTR0, crucial for white balance

  { -1, -1 }, // Terminate
};

static void allwrite(unsigned char *buf, int len) {
  int fd;
  int sent = 0;
  int rc;

  fd = open("/dev/xillybus_write_8", O_WRONLY);
  if (fd < 0) {
    perror("Failed to open /dev/xillybus_write_8 write-only");
    exit(1);
  }

  while (sent < len) {
    rc = write(fd, buf + sent, len - sent);

    if ((rc < 0) && (errno == EINTR))
      continue;

    if (rc < 0) {
      perror("write");
      exit(1);
    }

    if (rc == 0) {
      fprintf(stderr, "Reached write EOF (?!)\n");
      exit(1);
    }

    sent += rc;
  }

  close(fd); // This causes immediate flush of data
}

static void allread(int fd, unsigned char *buf, int len) {
  int received = 0;
  int rc;

  while (received < len) {
    rc = read(fd, buf + received, len - received);

    if ((rc < 0) && (errno == EINTR))
      continue;

    if (rc < 0) {
      perror("read");
      exit(1);
    }

    if (rc == 0) {
      fprintf(stderr, "Reached read EOF. Quitting.\n");
      exit(1);
    }

    received += rc;
  }
}


static void i2c_write(int addr, unsigned char data) {
  unsigned char sendbuf[3] = { i2c_addr << 1, addr, data };

  allwrite(sendbuf, sizeof(sendbuf));
}

static void i2c_read(int addr, unsigned char *data) {
  int fdr;

  unsigned char cmdbuf[2] = { i2c_addr << 1, addr };
  unsigned char dummybuf[2] = { (i2c_addr << 1) | 1, 0 };

  allwrite(cmdbuf, sizeof(cmdbuf));

  // We open xillybus_read_8 only now. Had it been open during the first
  // operation, there would have been a restart condition rather than a
  // stop condition after the first command.

  fdr = open("/dev/xillybus_read_8", O_RDONLY);

  if (fdr < 0) {
    perror("Failed to open /dev/xillybus_read_8 read-only");
    exit(1);
  }

  allwrite(dummybuf, sizeof(dummybuf));

  allread(fdr, data, sizeof(*data));

  close(fdr);
}

int main(int argc, char **argv) {
  int i;

  unsigned char value;

  int product_id = 0;
  unsigned char *p = (void *) &product_id;

  // Read product ID from sensor and verify that it's correct

  i2c_read(0x0b, p++);
  i2c_read(0x0a, p++);

  printf("Camera sensor's product ID is 0x%04x\n", product_id);

  if (product_id != 0x7673) {
    printf("Incorrect product ID. Stopping.\n");
    exit(1);
  }

  if (0) { // Change this in order to print out registers instead
    for (i=0; i<=0xc9; i++) {
      i2c_read(i, &value);

      printf("Reg 0x%02x = 0x%02x\n", i, value);
    }

    return 0;
  }

  for (i=0; writelist[i].addr >= 0; i++) {
    i2c_read(writelist[i].addr, &value);

    printf("Reg 0x%02x = 0x%02x%s\n", writelist[i].addr, value,
	   (value == writelist[i].value) ? "" : " (to be altered)");
  }

  for (i=0; writelist[i].addr >= 0; i++) {
    i2c_write(writelist[i].addr, writelist[i].value);

    printf("Wrote 0x%02x = 0x%02x\n",
	   writelist[i].addr, writelist[i].value);
  }

  return 0;
}
