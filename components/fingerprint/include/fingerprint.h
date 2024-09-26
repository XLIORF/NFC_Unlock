/*
 * @Author: 酒鸢 terraprism@163.com
 * @Date: 2024-06-22 22:13:32
 * @LastEditors: 酒鸢 terraprism@163.com
 * @LastEditTime: 2024-06-25 17:53:40
 * @FilePath: /fingerprint/components/fingerprint/include/fingerprint.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once 

typedef enum 
{
    FP_OK,
    FP_FAIL,
    FP_VERIFY_FAIL,
} fp_err_t;

void fp_init(void);
fp_err_t fp_add();
fp_err_t fp_rmove();
fp_err_t fp_verify(uint16_t *page_number);
fp_err_t fp_remove_all();