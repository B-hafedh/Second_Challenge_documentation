/* This code is the implementation of a custom bootloader for STM32 microcontroller that allows
   to do some tasks as uploading new firmware and upgrade image in user applications using USART protocol.*/


#include "main.h"                  

#define BOOT_FLAG_ADDRESS           0x08004000U // Bootloader address
#define APP_ADDRESS                 0x08008000U // Start address of user application (contains the main stack pointer MSP address).
#define TIMEOUT_VALUE               SystemCoreClock/4 

#define ACK     0x06U //Acknowledgment Byte ID.
#define NACK    0x16U //No Acknowledgment Byte ID.

static UART_HandleTypeDef huart; // UART handle structure instantiation.
static uint8_t RX_Buffer[32];    // Used to store the received data from the host.
// bootloader COMMANDs.
typedef enum
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;
/*Functions declaration*/
static void Jump2App(void);
static void Boot_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);

int main(void)
{
    Clk_Update();// Update the system clock.
    Boot_Init(); // Initialize bootloader.
    
    Transmit_ACK(&huart); // send ACK to the host --> ready to receive commands.
    // check if the transmission line is BUSY.
    if(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
    {
        Transmit_NACK(&huart); // send NACK to the host --> not ready to receive commmands.
        Jump2App(); // Jump to the user application.
    }
    //check if the host is ready to send data by checking for the ACK byte. 
    if(Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK)
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
    
	for(;;) // infinite loop --> to keep receiving commands. 
	{
        // receive 2 bytes of data (COMMAND code + checksum) from the host until the timeout exceeded.
        while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
        //check if the COMMAND code is received correctly.
        if(Check_Checksum(RX_Buffer, 2) != 1)
        {
            Transmit_NACK(&huart); // send a NACK byte to the host and the command is aborted.
        }
        // if the COMMAND code is correct --> the different commands can be performed using a switch case statement.
        //for each COMMAND code an ACK will be sent to the host and then the corresponding function will be executed.
        else
        {
            switch(RX_Buffer[0]) // RX_Buffer[0]= COMMAND code.
            {
                case ERASE:
                    Transmit_ACK(&huart); 
                    Erase();
                    break;
                case WRITE:
                    Transmit_ACK(&huart);
                    Write();
                    break;
                case CHECK:
                    Transmit_ACK(&huart);
                    Check();
                    break;
                case JUMP:
                    Transmit_ACK(&huart);
                    Jump2App();
                    break;
                default: 
                    Transmit_NACK(&huart);
                    break;
            }
        }
	}
    
    for(;;);
	return 0;
}
//This function performs the jump to the user application address and execute it.
static void Jump2App(void)
{ 
    // Check if the stack address is valid --> within the range of SRAM.
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000) 
    {
        __disable_irq(); // Disables interrupts.
        uint32_t jump_address = *(__IO uint32_t *)(APP_ADDRESS + 4); // get Reset_Handler function address-->the next address in vector table of the application.
        __set_MSP(*(__IO uint32_t *)APP_ADDRESS);                   //Initialize user application's Stack Pointer.
        //Run the Reset_Handler and the application function.
        void (*pmain_app)(void) = (void (*)(void))(jump_address);   
        pmain_app();
    }
    
}

// This function performs the Initialization of the bootloader.
static void Boot_Init(void)
{
    GPIO_InitTypeDef gpio_uart; 
    
    gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3; // Select the GPIO pins to be configured.
    gpio_uart.Mode = GPIO_MODE_AF_PP; // specify the operating mode: Alternate Function Push Pull mode.
    gpio_uart.Pull = GPIO_PULL_NONE; //  No pull-up or pull-down for the selected pins.
    gpio_uart.Speed = GPIO_SPEED_LOW; // set the pins' speed to 2 MHz.
    gpio_uart.Alternate = GPIO_AF7_USART2; // Select the peripheral to be connected to the selected pins (USART2).
    
    HAL_RCC_GPIOA_CLK_ENABLE(); // Enable the AHB1 peripheral clock (used for registers read/write access).
    HAL_GPIO_Init(GPIOA, &gpio_uart); // Initialize the GPIOA port pins with "gpio_uart" configuration. 
    //Configure the USART communication parameters.
    huart.Init.BaudRate = 115200; // set the communication baud rate to 115200.
    huart.Init.Mode = HAL_UART_MODE_TX_RX; // both receive and transmit modes are enabled.
    huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16; // set the Oversampling rate to 16-->(baudrate=input_freq/oversampling_rate).
    huart.Init.Parity = HAL_UART_PARITY_NONE; // No parity bit is used for errors detection.
    huart.Init.StopBits = HAL_UART_STOP_1; // use 1 bit as stop bit (indicates the end of the data frame).
    huart.Init.WordLength = HAL_UART_WORD8; // 8 bits of data will be received or transmitted in a frame.
    huart.Instance = USART2; // get the USART2 base addresses --> get access to USART2 registers.
    
    HAL_RCC_USART2_CLK_ENABLE(); // Enable the APB1 peripheral clock (used for registers read/write access).
    HAL_UART_Init(&huart); // Initializes the UART mode with "huart" handle parameters.
}

// will Send Acknowledgment bytes to the host (ready to receive commands).
static void Transmit_ACK(UART_HandleTypeDef *handle) 
{
    uint8_t msg[2] = {ACK, ACK}; // data buffer(contains 2 ACK bytes).
    
    HAL_UART_Tx(handle, msg, 2); //  transmit 2 acknoledgment bytes over UART.
}

// will Send No Acknowledge bytes to the host and aborts the command.
static void Transmit_NACK(UART_HandleTypeDef *handle) 
{
    uint8_t msg[2] = {NACK, NACK}; // data buffer(contains 2 NACK bytes).
    
    HAL_UART_Tx(handle, msg, 2); //  transmit 2 No acknoledgment bytes over UART.
}

// verify that the data contained in the packet was received correctly (using XOR operator).
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)
{
    uint8_t initial = 0xFF; // performs bytes innverting.
    uint8_t result = 0x7F; /* the bootloader code will begin after receiving 0x7F data frame:
                             (check AN3155 note for more information).*/
    
    result = initial ^ *pBuffer++; // Invert the checksum byte (the first element of the buffer).
    len--;
    // XORing all received bytes (data + checksum byte)--> the obtained output is inverted.
    while(len--)
    {
        result ^= *pBuffer++;
    }

    result ^= 0xFF; // invert the inverted checksum result.

    // verify that the XORed result = 0x00 --> the data was received correctly.
    if(result == 0x00) 
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
// this function allows the host to erase Flash memory sectors.
static void Erase(void)
{
    Flash_EraseInitTypeDef flashEraseConfig; 
    uint32_t sectorError; // Variable to store the config information on faulty sector in case of error.
    // receive 3 bytes of data(number of sectors + Initial sector + checksum) from the host until the timeout is exceeded.
    while(HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    //check if the checksum is correct.
    if(Check_Checksum(RX_Buffer, 3) != 1) // the checksum is not correct.
    {
        Transmit_NACK(&huart); // send a NACK byte to the host and the erase command is aborted.
        return;
    }
    
    if(RX_Buffer[0] == 0xFF) // Global erase request.
    {
        Transmit_NACK(&huart); // send a NACK byte to the host and the Erase command is aborted.
    }
    else // checksum is correct.
    {
        // set the Flash erase parameters. 
        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR; // select the erase_type: Sectors erase only.
        flashEraseConfig.NbSectors = RX_Buffer[0]; // Number of sectors to be erased.
        flashEraseConfig.Sector = RX_Buffer[1]; // Initial Flash sector to erase.

        HAL_Flash_Unlock(); // Get access to the Falsh control registers.
        HAL_Flash_Erase(&flashEraseConfig, &sectorError);// perform the erase of the specified Flash sectors.
        HAL_Flash_Lock(); // Lock the Flash control register access.
        
        Transmit_ACK(&huart); // send an ACK byte to the host.
    }
}

// This function allows to write in specified Flash memory space.
static void Write(void)
{
    uint8_t numBytes; // Variable to store the number of data bytes to be received.
    uint32_t startingAddress = 0; // Writing start address (4 bytes).
    uint8_t i;
    // receive 5 bytes of data (4 bytes start address + checksum) from the host until the timeout is exceeded.
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    //check if the checksum is correct.
    if(Check_Checksum(RX_Buffer, 5) != 1) // the checksum is not correct.
    {
        Transmit_NACK(&huart); // send a NACK byte to the host and the Write command is aborted.
        return;
    }
    else
    {
        Transmit_ACK(&huart); // send an ACK byte to the host.
    }
    // store the "startAddress" (separated 4 bytes) in a 32-bit format (using left shifting).
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    // receive 2 bytes of data (number of bytes + checksum) from the host until the timeout is exceeded.
    while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    numBytes = RX_Buffer[0];
    // receive data bytes to be written and checksum byte from the host until the timeout is exceeded.
    while(HAL_UART_Rx(&huart, RX_Buffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    //check if the checksum is correct.
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart); // send a NACK byte to the host and the Write command is aborted.
        return;
    }

    i = 0;
    HAL_Flash_Unlock(); // Get access to the Falsh control registers.
    while(numBytes--)
    {
        // Write the desired data bytes to Flash starting from startingAddress.
        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, RX_Buffer[i]); 
        startingAddress++;
        i++; 
    }
    HAL_Flash_Lock(); // Lock the Flash control register access.
    Transmit_ACK(&huart); // send an ACK byte to the host.
}
// This function performs the CRC check for a specified Flash memory space.
static void Check(void)
{
    uint32_t startingAddress = 0; // Memory space start address.
    uint32_t endingAddress = 0;   // Memory space end address.
    uint32_t address; // address counter.
    uint32_t *data; // pointer to the buffer containing the data to be checked.
    uint32_t crcResult; // Used to store the accumulated CRC of the data.

    // receive 5 bytes of data (4 bytes startingAddress + checksum) from the host unti the timeout exceeded.
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    //check if the checksum is correct.
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart); // send a NACK byte to the host and the Check command is aborted.
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    // store the "startAddress" (separated 4 bytes) in a 32-bit format (using left shifting).
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);

    // DO the same with the "endingAddress".
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    

    HAL_RCC_CRC_CLK_ENABLE();// Enable the integrated CRC peripheral clock.
    data = (uint32_t *)((__IO uint32_t*) startingAddress); // Initialize data pointer.
    // get all the data to be checked starting from "startingAddress".
    for(address = startingAddress; address < endingAddress; address += 4) 
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(data, 1); // Compute the accumulated 32-bit CRC of "data" based on the previous CRC value.
    }
    
    HAL_RCC_CRC_CLK_DISABLE(); // Disable CRC peripheral clock.
    // check if the accumulated CRC is correct.
    if(crcResult == 0x00)// CRC is correct
    {
        Transmit_ACK(&huart);// send an ACK byte to the host.
    }
    else
    {
        Transmit_NACK(&huart); // send a NACK byte to the host and the Check command is aborted.
    }
    
    Jump2App(); // Jump to the User application.
}
