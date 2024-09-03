#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/timex.h>

#include <pigpio.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdlib.h>

#define I2C_DEV_PATH "/dev/i2c-1"  // Use the correct I2C bus number
#define I2C_SLAVE_ADDR 0x40        // Replace with your I2C device's slave address
#define BITSTREAM_FILE "test.bit"  // The bitstream file
#define GPIO_PIN 4  // GPIO4

int i2c_fd;


// Set the timeout in milliseconds
int set_i2c_timeout(int file, int timeout_ms) {
    int timeout = timeout_ms;
    if (ioctl(file, I2C_TIMEOUT, timeout) < 0) {
        perror("Failed to set I2C timeout");
        return -1;
    }
    return 0;
}

// Function to initialize the I2C interface
int i2c_init() {

    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    // Set a longer timeout, e.g., 25 seconds (25000 milliseconds)
    if (set_i2c_timeout(i2c_fd, 10000) < 0) {
        close(i2c_fd);
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
int i2c_write_byte(int length, unsigned char *data_bytes){
    struct i2c_msg msgs[1];
    int num_msgs = 1;
    
    // First message: Write command bytes (without stop)
    msgs[0].addr = I2C_SLAVE_ADDR;
    msgs[0].flags = 0;  //Write
    msgs[0].len = length;
    msgs[0].buf = data_bytes;
    
    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = num_msgs;

    // Perform combined write and read operation
    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        perror("Failed to perform write I2C transaction");        
        return -1;
    }

    return 0;
} 

int i2c_write_long(unsigned char *write_command, int writecomm_len, unsigned char *write_data, int writedata_len) {
  struct i2c_rdwr_ioctl_data ioctl_data;  

    if (writedata_len<=8192 ){
        struct i2c_msg msgs[2];
        
        unsigned char *buffers[2] = {write_command, write_data};
        int lengths[2] = {writecomm_len, writedata_len};
        // Prepare the I2C messages using a loop
        for (int i = 0; i < 2; i++) {
            msgs[i].addr = I2C_SLAVE_ADDR;
            msgs[i].flags = 0;  // Write
            msgs[i].len = lengths[i];
            msgs[i].buf = buffers[i];
        }

        ioctl_data.msgs = msgs;
        ioctl_data.nmsgs = 2;
        
    }
    else {
        int chunk_size = 8192;  // Define the chunk size in bytes
        int num_bytes_command = writecomm_len;  // Length in bytes for command data
        int num_bytes_data = writedata_len;  // Length in bytes for data
        int total_len_bytes = num_bytes_command + num_bytes_data;  // Total length in bytes
        int num_chunks = (total_len_bytes + chunk_size - 1) / chunk_size;  // Calculate number of chunks

        struct i2c_msg msgs[num_chunks];  // Array to hold I2C messages

        // Combine write_command and write_data into a single buffer  
        unsigned char combined_buffer[total_len_bytes];
        memcpy(combined_buffer, write_command, num_bytes_command);  // Copy command data
        memcpy(combined_buffer + num_bytes_command, write_data, num_bytes_data);  // Copy data

        // Prepare the I2C messages using a loop
        for (int i = 0; i < num_chunks; i++) {
            msgs[i].addr = I2C_SLAVE_ADDR;
            msgs[i].flags = 0;  // Write

            // Calculate the size of the current chunk
            int current_chunk_size = (i == num_chunks - 1) ? (total_len_bytes % chunk_size) : chunk_size;
            printf("Chunk Size: %d\r\n", current_chunk_size);
            if (current_chunk_size == 0) current_chunk_size = chunk_size;  // Handle case where the last chunk is exactly chunk_size bytes

            msgs[i].len = current_chunk_size;
            msgs[i].buf = &combined_buffer[i * chunk_size];  // Point to the start of the current chunk in the buffer
            
            // Print the contents of msgs[i].buf

        }

        // Prepare the ioctl data structure
        ioctl_data.msgs = msgs;
        ioctl_data.nmsgs = num_chunks;
        printf("Num of Chunks: %d \r\n",num_chunks);
        
    }

    // Perform the combined write operation
    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        perror("Failed to perform combined I2C transaction");
        return -1;
    }

    return 0;
}

int i2c_read_byte(int length, unsigned char *a_ByteRead, int send_ack){
    struct i2c_msg msgs[1];
    int num_msgs = 1;
    
    // Second message: Read 4 bytes of data
    msgs[0].addr = I2C_SLAVE_ADDR;
    msgs[0].flags = I2C_M_RD;  // Read flag
    msgs[0].len = length;
    msgs[0].buf = a_ByteRead;


    struct i2c_rdwr_ioctl_data ioctl_data;
    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = num_msgs;

    // Perform combined write and read operation
    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        perror("Failed to perform read I2C transaction");
        return 0;
    }
    
    return 0;    
}

// Function to write and read data on the I2C bus
int i2c_write_and_read(unsigned char *write_buf, int write_len, unsigned char *read_buf, int read_len) {
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data ioctl_data;

    // prepare write message
    msgs[0].addr = I2C_SLAVE_ADDR;
    msgs[0].flags = 0; // write
    msgs[0].len = write_len;
    msgs[0].buf = write_buf;

    // prepare read message
    msgs[1].addr = I2C_SLAVE_ADDR;
    msgs[1].flags = 1; // read
    msgs[1].len = read_len;
    msgs[1].buf = read_buf;

    // prepare the ioctl data structure
    ioctl_data.msgs = msgs;
    ioctl_data.nmsgs = 2;
    if(ioctl(i2c_fd, I2C_RDWR, &ioctl_data)<0)
    {
        perror("Failed to perform combined I2C transaction");
        return -1;
    }

    return 0;
}

// Function to read the bitstream from a file
unsigned char* read_bitstream(const char *filename, size_t *bitstream_size) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("Failed to open bitstream file");
        return NULL;
    }

    // Get the size of the bitstream
    fseek(file, 0, SEEK_END);
    *bitstream_size = ftell(file);
    rewind(file);

    // Allocate memory for the bitstream
    unsigned char *bitstream = (unsigned char*)malloc(*bitstream_size);
    if (!bitstream) {
        perror("Failed to allocate memory for bitstream");
        fclose(file);
        return NULL;
    }

    // Read the bitstream into memory
    if (fread(bitstream, 1, *bitstream_size, file) != *bitstream_size) {
        perror("Failed to read bitstream file");
        free(bitstream);
        fclose(file);
        return NULL;
    }

    fclose(file);
    return bitstream;
}

void delay_ms(int milliseconds) {
    usleep(milliseconds * 1000);  // Convert milliseconds to microseconds
}

int main() {
    printf("Starting FPGA Configuration\r\n");

    unsigned char write_buf[4];
    unsigned char read_buf[4];
    size_t bitstream_size;
    unsigned char activation_key[5] = {0xFF, 0xA4, 0xC6, 0xF4, 0x8A};
    unsigned char *bitstream = read_bitstream(BITSTREAM_FILE, &bitstream_size);

    // Initialize the pigpio library
    if (gpioInitialise() < 0) {
        printf("pigpio initialization failed\n");
        return 1;
    }

    // Set GPIO_PIN as output
    gpioSetMode(GPIO_PIN, PI_OUTPUT);
        
    gpioWrite(GPIO_PIN, 0);  // Set GPIO_PIN to LOW
    delay_ms(1000);
    
    if (!bitstream) {
        printf("error reading bitstream in from file\r\n");
        return 1;  // Exit if bitstream loading fails
    }

    if (i2c_init() < 0) {
        printf("failed to initialize i2c\r\n");
        free(bitstream);
        return 1;  // Exit if I2C initialization fails
    }
    
    // Step 1: Initialize
    printf("Step 1: Send Activation Key\r\n");
    if (i2c_write_byte(5, activation_key) < 0) {
        printf("failed to send activation key\r\n");
        return 1;  // Exit if writing activation key fails
    }
    gpioWrite(GPIO_PIN, 1);  // Set GPIO_PIN to LOW
    delay_ms(10);

    // Step 2: Check IDCODE (Optional)
    printf("Step 2: Check IDCODE (Optional)\r\n");
    write_buf[0] = 0xE0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(write_buf, 4, read_buf, 4) < 0) {
        printf("failed to send IDCODE Command\r\n");
        free(bitstream);
        return 1;  // Exit if write/read fails
    }

    printf("Device ID read: ");
    for (int i = 0; i < 4; i++) {
        printf("%02X ", read_buf[i]);
    }
    printf("\r\n");

    // Step 3: Enable SRAM Programming Mode
    printf("Step 3: Enable SRAM Programming Mode\r\n");
    write_buf[0] = 0xC6; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(4, write_buf) < 0) {
        printf("failed to send SRAM Command\r\n");
        free(bitstream);
        return 1;  // Exit if writing fails
    }
    delay_ms(1);

    // Step 4: Erase SRAM
    printf("Step 4: Erase SRAM\r\n");
    write_buf[0] = 0x0E; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(4, write_buf) < 0) {
        printf("failed to send SRAM Erase Command\r\n");
        free(bitstream);
        return 1;  // Exit if writing fails
    }
    delay_ms(5000);  // Wait for erase to complete

    // Step 5: Read Status Register
    printf("Step 5: Read Status Register\r\n");
    write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(write_buf, 4, read_buf, 4) < 0) {
        printf("failed to send READ Status Command\r\n");
        free(bitstream);
        return 1;  // Exit if write/read fails
    }
    // Check status register bits according to the guide (e.g., Bit-12 Busy = 0, Bit-13 Fail = 0)

    printf("Status Register: ");
    for (int i = 0; i < 4; i++) {
        printf("%02X ", read_buf[i]);
    }
    printf("\r\n");

    // Step 6: Program SRAM
    printf("Step 6: Program SRAM\r\n");
    write_buf[0] = 0x46; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(4, write_buf) < 0) {
        printf("failed to send Program Command\r\n");
        free(bitstream);
        return 1;  // Exit if writing fails
    }

    // Step 7: Program SRAM
    printf("Step 7 & 8: Bitstream Burst & Shift bitstream in\r\n");
    write_buf[0] = 0x7A; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;

    // Program bitstream
    if (i2c_write_long(write_buf, 4, bitstream, bitstream_size)< 0) {
        printf("failed to send bitstream\r\n");
        free(bitstream);
        return 1;  // Exit if writing fails
    }
    
    delay_ms(10);

    // Step 9: Verify USERCODE (Optional)
    printf("Step 9: Verify USERCODE (Optional)\r\n");
    write_buf[0] = 0xC0; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(write_buf, 4, read_buf, 4) < 0) {
        printf("failed to send USERCODE Command\r\n");
        free(bitstream);
        return 1;  // Exit if write/read fails
    }
    // Compare read_buf with expected USERCODE if necessary

    printf("USERCODE: ");
    for (int i = 0; i < 4; i++) {
        printf("%02X ", read_buf[i]);
    }
    printf("\r\n");


    // Step 10: Read Status Register again
    printf("Step 10: Read Status Register\r\n");
    write_buf[0] = 0x3C; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_and_read(write_buf, 4, read_buf, 4) < 0) {
        printf("failed to send READ Status Command\r\n");
        free(bitstream);
        return 1;  // Exit if write/read fails
    }
    // Check status register bits again (e.g., Bit-8 DONE = 1, Bit-12 Busy = 0, Bit-13 Fail = 0)

    printf("Status Register: ");
    for (int i = 0; i < 4; i++) {
        printf("%02X ", read_buf[i]);
    }
    printf("\r\n");

    // Step 11: Exit Programming Mode
    printf("Step 11: Exit Programming Mode\r\n");
    write_buf[0] = 0x26; write_buf[1] = 0x00; write_buf[2] = 0x00; write_buf[3] = 0x00;
    if (i2c_write_byte(4, write_buf) < 0) {
        printf("failed to send Exit Command\r\n");
        free(bitstream);
        return 1;  // Exit if writing fails
    }

    close(i2c_fd);  // Close the I2C device file
    free(bitstream);  // Free the bitstream memory
    printf("FPGA Configuration Completed\r\n");
    return 0;
}
