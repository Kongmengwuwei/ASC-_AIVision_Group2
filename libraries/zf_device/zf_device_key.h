/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_device_key
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* ���߶��壺
*                   ------------------------------------
*                   ģ��ܽ�            ��Ƭ���ܽ�
*                   // һ�������尴����Ӧ������
*                   KEY1/S1             �鿴 zf_device_key.h �� KEY_LIST[0]
*                   KEY2/S2             �鿴 zf_device_key.h �� KEY_LIST[1]
*                   KEY3/S3             �鿴 zf_device_key.h �� KEY_LIST[2]
*                   KEY4/S4             �鿴 zf_device_key.h �� KEY_LIST[3]
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_key_h_
#define _zf_device_key_h_

#include "zf_common_debug.h"

#include "zf_driver_gpio.h"

// ���尴������ �û��������������޸� Ĭ�϶����ĸ�����
// ���尴��˳���Ӧ�·� key_index_enum ö�����ж����˳��
// ����û������������� ��ô��Ҫͬ�����·� key_index_enum ö��������������
#define KEY_LIST                    {C14, C15, C26, C27}

#define KEY_RELEASE_LEVEL           (GPIO_HIGH)                                 // ������Ĭ��״̬ Ҳ���ǰ����ͷ�״̬�ĵ�ƽ
#define KEY_MAX_SHOCK_PERIOD        (10       )                                 // �����������ʱ�� ��λ���� �������ʱ�����źŻᱻ��Ϊ���Ӳ�����
#define KEY_LONG_PRESS_PERIOD       (1000     )                                 // ��С����ʱ�� ��λ���� �������ʱ�����źŻᱻ��Ϊ�ǳ�������

typedef enum
{
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_NUMBER,
}key_index_enum;                                                                // �������� ��Ӧ�Ϸ�����İ������Ÿ��� Ĭ�϶����ĸ�����

typedef enum
{
    KEY_RELEASE,                                                                // �����ͷ�״̬
    KEY_SHORT_PRESS,                                                            // �����̰�״̬
    KEY_LONG_PRESS,                                                             // ��������״̬
}key_state_enum;

void            key_scanner             (void);
key_state_enum  key_get_state           (key_index_enum key_n);
void            key_clear_state         (key_index_enum key_n);
void            key_clear_all_state     (void);
void            key_init                (uint32 period);

#endif
