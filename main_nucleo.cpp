#include "mbed.h"
#include "messages.h"

DigitalOut myled(LED1);
Serial serial(SERIAL_TX, SERIAL_RX);
Ticker ticker;
InterruptIn mybutton(USER_BUTTON);

void Tx_interrupt();
void Rx_interrupt();
bool send_buffer(const uint8_t* msg, unsigned int size);
bool receive_message(uint8_t *msg_buf, int size, int *len);
bool send_message(const message *msg, uint8_t *buf, int size);
 
#define BUF_SIZE 255
 
char tx_buffer[BUF_SIZE];
char rx_buffer[BUF_SIZE];
 
// pointers to the circular buffers
volatile int tx_in = 0;
volatile int tx_out = 0;
volatile int rx_in = 0;
volatile int rx_out = 0;

#define MESSAGE_SIZE sizeof(message)

typedef struct // computation data
{
    uint8_t n,  current_chunk;
    double aR, aI; // start
    double lengthR, lengthI; // end
    double cR, cI; // parameter C
    double stepR, stepI; // size of step
    bool computing, computation_set;
}computation;

void tick() {
    myled = !myled;
}
int main() {
    serial.baud(115200);
    serial.attach(&Rx_interrupt, Serial::RxIrq); // attach interrupt handler to receive data
    serial.attach(&Tx_interrupt, Serial::TxIrq); // attach interrupt handler to transmit data

    for (int i = 0; i < 10; i++) {
        myled = !myled;
        wait(0.15);
    }   
    while (serial.readable())
        serial.getc(); 

    message msg = { .type = MSG_STARTUP, .data.startup.message = {'S','E','M','E','S','T','R','A', 'L'} };
    uint8_t msg_buf[MESSAGE_SIZE];
    int msg_len;
    
    computation parameters = {.n = 60, .cR = -0.4, .cI = 0.6, .aR = -1.6, .aI = -1.1, .lengthR = 1.1,
    .lengthI = 1.1, .computing = false, .computation_set = false, .current_chunk = 0};
    
    send_message(&msg, msg_buf, MESSAGE_SIZE);
    bool abort_request = false;
    bool computing = false;
    float period = 0.1;
    while (1)
    {
        if(mybutton == 0)
        {
            if (computing) //abort computing
            {
                msg.type = MSG_ABORT;
                send_message(&msg, msg_buf, MESSAGE_SIZE);
                computing = false;
                abort_request = false;
                ticker.detach();
                myled = 0;
            }
            else
            {
                msg.type = MSG_ERROR;
                send_message(&msg, msg_buf, MESSAGE_SIZE);
            }
            wait(0.1);
        }
        if (abort_request) 
        {
            if (computing) //abort computing
            {  
                msg.type = MSG_ABORT;
                send_message(&msg, msg_buf, MESSAGE_SIZE);
                computing = false;
                abort_request = false;
                ticker.detach();
                myled = 0;
            }
            else
            {
                msg.type = MSG_ERROR;
                send_message(&msg, msg_buf, MESSAGE_SIZE);
            }
        }
        if (rx_in != rx_out) // something is in the receive buffer
        { 
            if (receive_message(msg_buf, MESSAGE_SIZE, &msg_len)) 
            {
                if (parse_message_buf(msg_buf, msg_len, &msg)) 
                {
                    switch(msg.type) 
                    {
                        case MSG_GET_VERSION:
                            msg.type = MSG_VERSION;
                            msg.data.version.major  = 0;
                            msg.data.version.minor  = 1;
                            msg.data.version.patch  = 2;
                            send_message(&msg, msg_buf, MESSAGE_SIZE);
                            break;
                        case MSG_ABORT:
                            msg.type = MSG_OK;
                            send_message(&msg, msg_buf, MESSAGE_SIZE);
                            computing = false;
                            abort_request = false;
                            ticker.detach();
                            myled = 0;
                            break;
                        case MSG_COMPUTE:
                            parameters.current_chunk = msg.data.compute.cid;
                            parameters.lengthR = msg.data.compute.n_re;
                            parameters.lengthI = msg.data.compute.n_im;
                            parameters.aR = msg.data.compute.re;
                            parameters.aI = msg.data.compute.im;
                            computing = true;
                            msg.type = MSG_OK;
                            send_message(&msg, msg_buf, MESSAGE_SIZE);
                            break;
                        case MSG_SET_COMPUTE:
                            parameters.cR = msg.data.set_compute.c_re;
                            parameters.cI = msg.data.set_compute.c_im;
                            parameters.stepR = msg.data.set_compute.d_re;
                            parameters.stepI = msg.data.set_compute.d_im;
                            parameters.n = msg.data.set_compute.n;
                            parameters.computation_set = true;
                            msg.type = MSG_OK;
                            send_message(&msg, msg_buf, MESSAGE_SIZE);
                            break;
                    } // end switch
                    
                } 
                else // message has not been parsed send error
                { 
                    msg.type = MSG_ERROR;
                    send_message(&msg, msg_buf, MESSAGE_SIZE);
                }
            } // end message received
        } // end if receive buffer
        else if (computing) 
        {
            ticker.attach(tick, period);
            int counter = 0;
            msg.type = MSG_COMPUTE_DATA;
            for (int i = 0; i < parameters.lengthI; i++)
            {
                for (int j = 0; j < parameters.lengthR; j++)
                {
                    double ZReal = parameters.aR + (parameters.lengthR - j) * parameters.stepR;
                    double ZImag = parameters.aI + i * parameters.stepI;
                    double OldZReal, OldZImag;
                    uint8_t iteration = 0;
                    // computation for each pixel
                    while (iteration < parameters.n && (ZReal*ZReal + ZImag*ZImag) < 4) // sqrt(2) = 4
                    {
                        OldZReal = ZReal;
                        OldZImag = ZImag;
                        ZReal = OldZReal * OldZReal - OldZImag * OldZImag + parameters.cR;
                        ZImag = 2 * OldZReal * OldZImag + parameters.cI;
                        iteration++;
                        counter++;
                    }
                    // coloring according to number of iterations
                    msg.data.compute_data.cid = parameters.current_chunk;
                    msg.data.compute_data.i_re = (parameters.lengthR - j);
                    msg.data.compute_data.i_im = i;
                    msg.data.compute_data.iter = iteration;
                    send_message(&msg, msg_buf, MESSAGE_SIZE);
                }
            }
            computing = false;
            msg.type = MSG_DONE;
            send_message(&msg, msg_buf, MESSAGE_SIZE);
            ticker.detach();
            myled = 0;
        } 
        else { sleep(); /* put the cpu to sleep mode, it will be wakeup on interrupt*/ }
    } // end while (1)
    
}



bool send_message(const message *msg, uint8_t *buf, int size) {
    return fill_message_buf(msg, buf, MESSAGE_SIZE, &size)
                        && send_buffer(buf, size);
}

void Tx_interrupt()
{
    // send a single byte as the interrupt is triggered on empty out buffer 
    if (tx_in != tx_out) {
        serial.putc(tx_buffer[tx_out]);
        tx_out = (tx_out + 1) % BUF_SIZE;
    } else { // buffer sent out, disable Tx interrupt
        USART2->CR1 &= ~USART_CR1_TXEIE; // disable Tx interrupt
    }
    return;
}

void Rx_interrupt()
{
    // receive bytes and stop if rx_buffer is full
    while ((serial.readable()) && (((rx_in + 1) % BUF_SIZE) != rx_out)) {
        rx_buffer[rx_in] = serial.getc();
        rx_in = (rx_in + 1) % BUF_SIZE;
    }
    return;
}

bool send_buffer(const uint8_t* msg, unsigned int size)
{
    if (!msg && size == 0) {
        return false;    // size must be > 0
    }
    int i = 0;
    NVIC_DisableIRQ(USART2_IRQn); // start critical section for accessing global data
    USART2->CR1 |= USART_CR1_TXEIE; // enable Tx interrupt on empty out buffer
    bool empty = (tx_in == tx_out);
    while ( (i == 0) || i < size ) { //end reading when message has been read
        if ( ((tx_in + 1) % BUF_SIZE) == tx_out) { // needs buffer space
            NVIC_EnableIRQ(USART2_IRQn); // enable interrupts for sending buffer
            while (((tx_in + 1) % BUF_SIZE) == tx_out) {
                /// let interrupt routine empty the buffer
            }
            NVIC_DisableIRQ(USART2_IRQn); // disable interrupts for accessing global buffer
        }
        tx_buffer[tx_in] = msg[i];
        i += 1;
        tx_in = (tx_in + 1) % BUF_SIZE;
    } // send buffer has been put to tx buffer, enable Tx interrupt for sending it out
    USART2->CR1 |= USART_CR1_TXEIE; // enable Tx interrupt
    NVIC_EnableIRQ(USART2_IRQn); // end critical section
    return true;
}

bool receive_message(uint8_t *msg_buf, int size, int *len)
{
    bool ret = false;
    int i = 0;
    *len = 0; // message size
    NVIC_DisableIRQ(USART2_IRQn); // start critical section for accessing global data
    while ( ((i == 0) || (i != *len)) && i < size ) {
        if (rx_in == rx_out) { // wait if buffer is empty
            NVIC_EnableIRQ(USART2_IRQn); // enable interrupts for receing buffer
            while (rx_in == rx_out) { // wait of next character
            }
            NVIC_DisableIRQ(USART2_IRQn); // disable interrupts for accessing global buffer
        }
        uint8_t c = rx_buffer[rx_out];
        if (i == 0) { // message type
            if (get_message_size(c, len)) { // message type recognized
                msg_buf[i++] = c;
                ret = *len <= size; // msg_buffer must be large enough
            } else {
                ret = false;
                break; // unknown message
            }
        } else {
            msg_buf[i++] = c;
        }
        rx_out = (rx_out + 1) % BUF_SIZE;
    }
    NVIC_EnableIRQ(USART2_IRQn); // end critical section
    return ret;
}
