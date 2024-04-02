#include "SortAver_Filter.h"
/*******************************************************************************
* 函  数 ：float  SortAver_Filter(float value)
* 功  能 ：去最值平均值滤波一组数据
* 参  数 ：value 采样的数据
*		   *filter 滤波以后的数据地址
* 返回值 ：无
* 备  注 : 无
*******************************************************************************/
void  SortAver_Filter(float value,float *filter,uint8_t n)
{
	static float buf[25] = {0.0};
	static uint8_t cnt =0,flag = 1;
	float temp=0;
	uint8_t i=0;
	buf[cnt++] = value;
	if(cnt<n && flag) 
		return;  //数组填不满不计算	
	else flag=0; 
	QuiteSort(buf,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp += buf[i];
	 }

	 if(cnt>=n) cnt = 0;

	 *filter = temp/(n-2);
}


/*******************************************************************************
* 函  数 ：void QuiteSort(float* a,int low,int high)
* 功  能 ：快速排序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：无
* 备  注 : 无
*******************************************************************************/
 void QuiteSort(float* a,int low,int high)
 {
     int pos;
     if(low<high)
     {
         pos = FindPos(a,low,high); //排序一个位置
         QuiteSort(a,low,pos-1);    //递归调用
         QuiteSort(a,pos+1,high);
     }
 }

 
 /*******************************************************************************
* 函  数 ：float FindPos(float*a,int low,int high)
* 功  能 ：确定一个元素位序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：返回元素的位序low
* 备  注 : 无
*******************************************************************************/
float FindPos(float*a,int low,int high)
{
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                       //如果右边的数大于VAL下标往前移
             a[low] = a[high];             //当右边的值小于VAL则复值给A[low]

        while(low<high && a[low]<=val)
             low++;                        //如果左边的数小于VAL下标往后移
             a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;
    return low;
}
