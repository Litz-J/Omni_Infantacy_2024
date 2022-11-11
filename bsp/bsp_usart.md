# bsp_usart

<p align='right'>neozng1@hnu.edu.cn</p>

> TODO:为初始化定义一个结构体`usart_init_config`用于保存初始化所需的参数从而避免单独赋值，使得整体风格统一。

## 代码结构

.h文件内包括了外部接口和类型定义,以及模块对应的宏。c文件内为私有函数和外部接口的定义。

### 类型定义

```c
#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 128 // if your protocol needs bigger buff, modify here

typedef void (*usart_module_callback)();

/* usart_instance struct,each app would have one instance */
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} usart_instance;
```

- `DEVICE_USART_CNT`是开发板上可用的串口数量。

- `USART_RXBUFF_LIMIT`是串口单次接收的数据长度上限，暂时设为128，如果需要更大的buffer容量，修改该值。

- `usart_module_callback()`是模块提供给串口接收中断回调函数使用的协议解析函数指针。对于每个需要串口的模块，需要定义一个这样的函数用于解包数据。

- 每定义一个`usart_instance`，就代表一个串口的**实例**（对象）。一个串口实例内有接收buffer，单个数据包的大小，该串口对应的`HAL handle`（代表其使用的串口硬件具体是哪一个）以及用于解包数据的回调函数。


### 外部接口

```c
void USARTRegister(usart_instance *_instance);
void USARTSend(usart_instance *_instance, uint8_t *send_buf, uint16_t send_size);
```

- `USARTRegister`是用于初始化串口对象的接口，module层的模块对象（也应当为一个结构体）内要包含一个`usart_instance`。

  **在调用该函数之前，需要先对其成员变量`*usart_handle`,`module_callback()`以及`recv_buff_size`进行赋值。**

- `USARTSend()`是通过模块通过其拥有的串口对象发送数据的接口，调用时传入的参数为串口实例指针，发送缓存以及此次要发送的数据长度（8-bit\*n)。

### 私有函数和变量

在.c文件内设为static的函数和变量

```c
static usart_instance *instance[DEVICE_USART_CNT];
```

这是bsp层管理所有串口实例的入口。

```c
static void USARTServiceInit(usart_instance *_instance)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
```

- `USARTServiceInit()`会被`USARTRegister()`调用，开启接收中断

- `HAL_UARTEx_RxEventCallback()`和`HAL_UART_ErrorCallback()`都是对HAL的回调函数的重定义（原本的callback是`__week`修饰的弱定义），前者在发生**IDLE中断**或**单次DMA传输中断**后会被调用（说明收到了完整的一包数据），随后在里面根据中断来源，调用拥有产生了该中断的模块的协议解析函数进行数据解包；后者在串口传输出错的时候会被调用，重新开启接收。