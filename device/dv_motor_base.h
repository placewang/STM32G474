#ifndef DV_MOTOR_BASE_H
#define DV_MOTOR_BASE_H

#ifdef __cplusplus
extern "C" {
#endif



struct motor_base;

struct motor_ops 
{
	int (*Enable)(struct  motor_base *mtr);                 
	int (*Disable)(struct motor_base *mtr);         
};


struct motor_base 
{
	const struct motor_ops *ops;     /* 第一个字段, 对象起始地址处 */
};

int dv_motor_base_init(struct motor_base *me, const struct motor_ops *ops);

int dv_motor_Enable(struct motor_base *me);
int dv_motor_Disable(struct motor_base *me);







#ifdef __cplusplus
}
#endif


#endif 



