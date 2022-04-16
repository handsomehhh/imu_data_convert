#pragma once
#ifndef _COMMUNICATOR_H_
#define _COMMUNICATOR_H_

#include "message.h"
#include <sys/mman.h>
#include <sys/sem.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

using namespace std;

/**
 * @brief used to define share memory
 * @param fd：共享内存的句柄
 * @param point：共享内存区域首地址
 * 
 */
struct MemoryDefinition
{
    int fd;
    void * point;
};

/**
 * @brief Create a Share Memory, and return a MemoryDefinition pointer.
 * 
 * @param fileName：共享内存的映射文件的名称
 * @param size：共享内存文件大小
 * @return MemoryDefinition* 
 */
MemoryDefinition* createShareMemory(const char* fileName, size_t size);


/**
 * @brief Connect a Share Memory, and return a MemoryDefinition pointer.
 * 
 * @param fileName 映射文件名称
 * @param size 文件大小
 * @return MemoryDefinition*  
 */
MemoryDefinition* connectShareMemory(const char* fileName, size_t size);

/**
 * @brief unmap between the noraml file and the share memory
 * 
 * @param definition a MemoryDefinition object used to describe a specific share memory
 */
void unMap(const MemoryDefinition* definition, size_t size);

//这里要修改，改成直接传入共享内存大小
MemoryDefinition* createShareMemory(const char* fileName, size_t write_size)
{
    int fd = open(fileName, O_CREAT | O_RDWR | O_TRUNC, 00777);
    if (fd < 0) {
        cout << "create file error!" << endl;
        return NULL;
    }

    //size_t write_size = sizeof(ExampleList);

    ftruncate(fd, write_size);

    void* point = mmap(NULL, write_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    MemoryDefinition* result = new MemoryDefinition;
    result->fd = fd;
    result->point = point;

    // cout <<"[" <<fileName << "]" << "...Finish create share Memory!" << endl;
    //cout << "...Finish create share Memory!" << endl;


    return result;
}

MemoryDefinition* connectShareMemory(const char* fileName, size_t write_size)
{
    MemoryDefinition* result = new MemoryDefinition;
    int fd = open(fileName, O_RDONLY, 00777);
    if (fd < 0) {
        cout << "open file error!" << endl;
        result->fd = -999;
        return result;
        // return NULL;
    }

    //size_t write_size = sizeof(ExampleList);

    void* point = mmap(NULL, write_size, PROT_READ, MAP_SHARED, fd, 0);

    
    result->fd = fd;
    result->point = point;

    // cout <<"[" <<fileName << "]" << "...Finish connect share Memory!" << endl;
    //cout << "...Finish connect share Memory!" << endl;

    return result;
}

void unMap(const MemoryDefinition* definition, size_t write_size)
{
    //size_t write_size = sizeof(ExampleList);
    munmap(definition->point, write_size);
    close(definition->fd);

    // cout << "...Finish unmap share Memory and close the normal file!" << endl;
}



#endif