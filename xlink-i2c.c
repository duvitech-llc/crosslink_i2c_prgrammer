#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#define I2C_DEV_PATH "/dev/i2c-1"  // Use the correct I2C bus number
#define I2C_SLAVE_ADDR 0x40        // Replace with your I2C device's slave address

int i2c_fd;

// Function to initialize the I2C interface
int i2c_init() {
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, I2C_SLAVE_ADDR) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(i2c_fd);
        return -1;
    }
    return 0;
}

// Function to write a byte array to the I2C bus
int i2c_write_byte(int length, unsigned char *data) {
    if (write(i2c_fd, data, length) != length) {
        perror("Failed to write to the i2c bus");
        return -1;
    }
    return 0;
}

// Function to write and read data on the I2C bus
int i2c_write_and_read(unsigned char *write_buf, int write_len, unsigned char *read_buf, int read_len) {
    if (write(i2c_fd, write_buf, write_len) != write_len) {
        perror("Failed to write to the i2c bus");
        return -1;
    }

    if (read(i2c_fd, read_buf, read_len) != read_len) {
        perror("Failed to read from the i2c bus");
        return -1;
    }
    return 0;
}


int main() {
    printf("Example I2C to CrossLink\r\n");

    unsigned char write_buf[4] = {0xE0, 0x00, 0x00, 0x00};
    unsigned char activation_key[5] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
    unsigned char read_buf[4];

    unsigned char global_write_buf[4];
    int global_length = 4;

    if (i2c_init() < 0) {
        return 1;  // Exit if I2C initialization fails
    }

    memcpy(global_write_buf, write_buf, sizeof(write_buf));

    if (i2c_write_byte(5, activation_key) < 0) {
        return 1;  // Exit if writing activation key fails
    }

    printf("Sent data: ");
    for (int i = 0; i < 4; i++) {
        printf("%02X ", global_write_buf[i]);
    }
    printf("\r\n");

    if (i2c_write_and_read(global_write_buf, global_length, read_buf, 4) < 0) {
        return 1;  // Exit if write/read fails
    }

    printf("Device ID read: ");
    for (int i = 0; i < 4; i++) {
        printf("%02X ", read_buf[i]);
    }
    printf("\r\n");

    close(i2c_fd);  // Close the I2C device file
    return 0;
}
