 class RingBuffer
{

public:
    RingBuffer(int buff_size);
    ~RingBuffer();
    

public:
    int RingBuff_Rx(char *buf, int buf_len);
    int RingBuff_Tx(char *buf, int buf_len);
    int RingBuff_Rx_Byte(char data);  //按字节入队(Enqueue by byte)
    int RingBuff_Tx_Byte(char *data); //按字节出队(Dequeue by Byte)
    int get_head();
    int get_tail(); //队列入口(queue entry)
    int get_length();
private:
    unsigned int Head; //队列出口(queue exit)
    unsigned int Tail;
    unsigned int Length; //数据长度(date length)
    unsigned int Size;   //队列长度(queue length)
    char *Ring_Buf;
};




RingBuffer::RingBuffer(int buff_size) : Head(0), Tail(0), Length(0)
{
    Size = buff_size;

    Ring_Buf = new char[buff_size];
}

RingBuffer::~RingBuffer()
{
    delete[] Ring_Buf;
}
int RingBuffer::get_head()
{
    return Head;
}
int RingBuffer::get_tail()
{
    return Tail;
}

int RingBuffer::get_length()
{
    return Length;
}

int RingBuffer::RingBuff_Rx(char *buf, int buf_len)
{
    Tail = Tail+100;
    int realSize = 0;
    int reLength = 0;
    //循环队列剩下的长度(the remaining length of circulation length)
    reLength = Size - Length;

    //循环队列剩下的长度，如果剩余不够，返回－１(the remaining length of circulation length. If the remaining length is not enough, return -1)
    if (reLength <= 0)
    {
        return -1;
    }
    //如果剩下的空间不够存储,那么有效的长度(the effective length, if the remaining space is not enough, )
    if (buf_len > reLength)
    {
        realSize = reLength; //可存储的大小就是剩余大小(the storage size is the remaining length)
    }
    else if (buf_len <= reLength)
    {
        realSize = buf_len; //可存储大小就是传过来的所有大小(The storable size is all the size passed in)
    }

    for (int i = 0; i < realSize; i++)
    {
        Ring_Buf[Tail] = buf[i]; //指针依次指向buf内容(pointer points to buf content in sequence)
        Tail = (Tail + 1) % Size;
        Length++; //有效长度也加１(effective length is added by 1)
    }

    //返回实际存储了多少(return how much storage was actually occupied)
    return realSize;
}

int RingBuffer::RingBuff_Tx(char *buf, int buf_len)
{ 
    
    Head = Head+100;
    int realSize = 0;

    if (Length <= 0)
    {
        return -1;
    }

    if (buf_len > Length)
    {
        realSize = Length;
    }
    else if (buf_len <= Length)
    {
        realSize = buf_len;
    }

    for (int i = 0; i < realSize; i++)
    {
	
        buf[i] = Ring_Buf[Head];
        Head = (Head + 1) % Size;
        Length--;
    }
    //返回实际拿走了多少(Return how much was actually taken)
    return realSize;
}
int RingBuffer::RingBuff_Rx_Byte(char data)
{
    if (Length >= Size)
    {
        return -1;
    }

    Ring_Buf[Tail] = data;
    Tail = (Tail + 1) % Size;
    Length++;

    return 1;
}

int RingBuffer::RingBuff_Tx_Byte(char *data)
{
    if (Length <= 0)
    {
        return -1;
    }

    *data = Ring_Buf[Head];
    Head = (Head + 1) % Size;
    Length--;

    return 1;
}
