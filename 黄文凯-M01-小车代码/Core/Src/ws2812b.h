#ifndef __WS2812B_H_
#define __WS2812B_H_

#include "main.h"

typedef struct				//��ɫ�ṹ��
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
}RGBColor_TypeDef;

#define RGB_NUM    3	// RGB����

// ��λ����
void RGB_RST(void);
// ��ɫ���ú���
void RGB_Set_Color(uint8_t LedId, RGBColor_TypeDef Color);
// RGB ˢ�º���
void RGB_Reflash(uint8_t reflash_num);
	
// ������ɫ����
void RGB_RED(uint16_t RGB_LEN);		//��
void RGB_GREEN(uint16_t RGB_LEN);		//��
void RGB_BLUE(uint16_t RGB_LEN);		//��
void RGB_YELLOW(uint16_t RGB_LEN);		//��
void RGB_MAGENTA(uint16_t RGB_LEN);	//��
void RGB_BLACK(uint16_t RGB_LEN);		//��
void RGB_WHITE(uint16_t RGB_LEN);		//��


#endif
