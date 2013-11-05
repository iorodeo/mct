#ifndef RING_BUFFER_H
#define RING_BUFFER_H

template <class T, unsigned int maxSize>
class RingBuffer
{
    public:
        RingBuffer();
        void add(T item);
        void empty();
        bool isEmpty();
        bool isFull();
        T getItemNewest();
        T getItemOldest();
        bool getItemByPos(unsigned int pos, T &item);
        bool getItemByAge(unsigned int age, T &item);
        unsigned int getSize();
        unsigned int getPos();
        unsigned int getNextPos();
        unsigned int getMaxSize();

    private:
        T buf_[maxSize];
        bool isFull_;
        bool isEmpty_;
        unsigned int nextPos_;
};

template <class T, unsigned int maxSize>
RingBuffer<T,maxSize>::RingBuffer()
{
    empty();
}

template <class T, unsigned int maxSize>
void RingBuffer<T,maxSize>::add(T item)
{
    buf_[nextPos_] = item;
    isEmpty_ = false;
    nextPos_++;
    if (nextPos_ >= maxSize)
    {
        nextPos_ = 0;
        isFull_ = true;
    }
}

template <class T, unsigned int maxSize>
void RingBuffer<T,maxSize>::empty()
{
    isFull_ = false;
    isEmpty_ = true;
    nextPos_ = 0;
}

template <class T, unsigned int maxSize>
bool RingBuffer<T,maxSize>::isEmpty()
{
    return isEmpty_;
}

template <class T, unsigned int maxSize>
bool RingBuffer<T,maxSize>::isFull()
{
    return isFull_;
}

template <class T, unsigned int maxSize>
bool RingBuffer<T,maxSize>::getItemByPos(unsigned int pos, T &item)
{
    bool status = false;
    if (~isEmpty_)
    {
        unsigned int size = getSize();
        if (pos < size)
        {
            item = buf_[pos];
            status = true;
        }
    }
    return status;
}

template <class T, unsigned int maxSize>
bool RingBuffer<T,maxSize>::getItemByAge(unsigned int age, T &item)
{
    bool status = false;
    unsigned int curPos = getPos();
    unsigned int size = getSize();

    if (age < size)
    {
        unsigned int itemPos;
        if (age > curPos)
        {
            itemPos = maxSize-(age-curPos);

        }
        else
        {
            itemPos = curPos-age;
        }
        item = buf_[itemPos];
        status = true;
    } 
    return status;
}

template <class T, unsigned int maxSize>
T RingBuffer<T,maxSize>::getItemNewest()
{
    static T dummyItem;
    T item = dummyItem;
    if (!isEmpty_)
    {
        unsigned int pos = getPos();
        getItemByPos(pos,item);
    }
    return item;
}

template <class T, unsigned int maxSize>
T RingBuffer<T,maxSize>::getItemOldest()
{
    static T dummyItem;
    T item = dummyItem;
    if (isFull_)
    {
         getItemByPos(nextPos_,item);
    }
    else
    {
         getItemByPos(0,item);
    }
    return item;
}

template <class T, unsigned int maxSize>
unsigned int RingBuffer<T,maxSize>::getSize()
{
    if (isFull_)
    {
        return maxSize;
    }
    else
    {
        return nextPos_;
    }
}

template <class T, unsigned int maxSize>
unsigned int RingBuffer<T,maxSize>::getPos()
{
    if (nextPos_ == 0)
    {
        return maxSize-1;
    }
    else
    {
        return nextPos_-1;
    }
}

template <class T, unsigned int maxSize>
unsigned int RingBuffer<T,maxSize>::getNextPos()
{
    return nextPos_;
}

template <class T, unsigned int maxSize>
unsigned int RingBuffer<T, maxSize>::getMaxSize()
{
    return maxSize;
}


#endif
