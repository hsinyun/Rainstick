#define WAVELET_LEN 100
/*Mathab command:
step=100; final=pi; max=40; 
a=[0:final/step:final-final/step];b=round(max*sin(a));
mystring='';for n=1:step mystring=sprintf('%s%d,',mystring, b(n));end;mystring
*/

static int wavelet[WAVELET_LEN] = {
		0,1,3,4,5,6,7,9,10,11,12,14,15,16,17,18,19,20,21,22,24,25,25,26,27,28,29,30,31,32,32,33,34,34,35,36,36,37,37,38,38,38,39,39,39,40,40,40,40,40,40,40,40,40,40,40,39,39,39,38,38,38,37,37,36,36,35,34,34,33,32,32,31,30,29,28,27,26,25,25,24,22,21,20,19,18,17,16,15,14,12,11,10,9,7,6,5,4,3,1,
};
