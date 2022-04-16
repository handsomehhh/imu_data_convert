#pragma once
#ifndef _COMMUNICATOR1_H_
#define _COMMUNICATOR1_H_
#include <stdio.h> 
#include <stdlib.h> 
#include <sys/types.h> 
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/sem.h> 
#include <sys/msg.h> 
#include <cstring>
#include <string>
#include "message.h"
using namespace std;
#define BUFSZ 256 
 
/**
 * @brief used to define share memory
 * @param fd：共享内存区域ID
 * @param point：共享内存区域首地址
 * 
 */
struct MemoryDefinition
{
    int fd;
    void* point;
};


key_t get_id(const string fileName)
{
    key_t ret = 0;
    for(int i = 0; i < fileName.size(); i++)
    {
        ret += fileName[i];
    }
     //cout<<fileName<<' '<<ret<<endl;
    return ret + 1000000;
}

/**
 * @brief Create a shm object
 * 
 * @param shm_key 共享内存键值
 * @param shm_num 共享内存字节长度
 * @param shm_flag 权限 （默认权限为读写权限）
 * @return void* 
 */
MemoryDefinition* createShareMemory(const string shm_key, size_t shm_num)//, int shm_flag = 1)
{
    //cout<<"size "<<shm_num<<endl;
    MemoryDefinition* shm_point = new MemoryDefinition();;
    int shm_flag = IPC_CREAT | 0666;
    int fd = shmget(get_id(shm_key), shm_num, shm_flag);
    void* point;
    if(fd != -1) {
        //
        point = shmat(fd, 0, 0);
        if((int*)point != (int*)-1) {
            //初始化
            memset(point, 0, shm_num);


            shm_point->fd = fd;
            shm_point->point = point;
            return shm_point;
        } else {
            perror("get shareMemory error1");
            //exit(EXIT_FAILURE); 
        }
    } else {
        perror("get shareMemory error2");
        //exit(EXIT_FAILURE); 
    }
    return shm_point;
}

/**
 * @brief 连接共享内存
 * 
 * @param shm_key 共享内存键值
 * @param shm_num 共享内存字节长度
 * @param shm_flag 权限 （默认权限为读写权限）
 * @return void* 
 */
MemoryDefinition* connectShareMemory(const string shm_key, size_t shm_num)//, int shm_flag = 1)
{
    //cout<<"size "<<shm_num<<endl;
    MemoryDefinition* shm_point = new MemoryDefinition();
    int shm_flag = IPC_CREAT | 0666;
    int fd = shmget(get_id(shm_key), shm_num, shm_flag);
    void* point;
    if(fd != -1) {
        //
        point = shmat(fd, 0, 0);
        if((int*)point != (int*)-1) {
            shm_point->fd = fd;
            shm_point->point = point;
            return shm_point;
        } else {
            perror("get shareMemory error3");
            //exit(EXIT_FAILURE); 
        }
    } else {
        perror("get shareMemory error4");
        //exit(EXIT_FAILURE); 
    }
    return shm_point;
}

void unMap(const MemoryDefinition* definition)
{
    //size_t write_size = sizeof(ExampleList);
    shmdt(definition->point);

    // cout << "...Finish unmap share Memory and close the normal file!" << endl;
}

#endif