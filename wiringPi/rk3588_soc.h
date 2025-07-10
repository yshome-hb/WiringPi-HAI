#ifndef	__RK3588_SOC_H__
#define	__RK3588_SOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RK3588_GPIO_NUM                         (0x40)
#define RK3588_GPIO_BIT(x)                      (1UL << (x))

//gpio0~gpio4 register base addr
#define RK3588_GPIO0_BASE                       0xfd8a0000U
#define RK3588_GPIO1_BASE                       0xfec20000U
#define RK3588_GPIO2_BASE                       0xfec30000U
#define RK3588_GPIO3_BASE                       0xfec40000U
#define RK3588_GPIO4_BASE                       0xfec50000U

//gpio offset
#define RK3588_GPIO_SWPORT_DR_L_OFFSET          0x00U
#define RK3588_GPIO_SWPORT_DR_H_OFFSET          0x04U
#define RK3588_GPIO_SWPORT_DDR_L_OFFSET         0x08U
#define RK3588_GPIO_SWPORT_DDR_H_OFFSET         0x0cU
#define RK3588_GPIO_EXT_PORT_OFFSET             0x70U

//CRU clock-controller base addr
#define RK3588_CRU_BASE                         0xfd7c0000U
#define RK3588_CRU_GATE_CON15                   (RK3588_CRU_BASE + 0x083CU)    //for pwm123_clk_en
#define RK3588_CRU_GATE_CON16                   (RK3588_CRU_BASE + 0x0840U)    //for gpio1 bit 14 15    30 31
#define RK3588_CRU_GATE_CON17                   (RK3588_CRU_BASE + 0x0844U)    //for gpio2/3/4 - bit 0 1 2 3 4 5    16 17 18 19 20 21
#define RK3588_CRU_GATE_CON19                   (RK3588_CRU_BASE + 0x084CU)    //for busioc_clk_en

#define RK3588_PMU1CRU_BASE                     0xfd7f0000U
#define RK3588_PMU1CRU_GATE_CON1                (RK3588_PMU1CRU_BASE + 0x0804U)    //for pmu1pwm_clk_en
#define RK3588_PMU1CRU_GATE_CON5                (RK3588_PMU1CRU_BASE + 0x0814U)    //for gpio0 - bit 5 6    21 22

//gpio iomux
#define RK3588_PMU1_IOC_BASE                    0xfd5f0000U
#define RK3588_PMU1_IOC_GPIO0A_IOMUX_SEL_L      (RK3588_PMU1_IOC_BASE + 0x00U)    //gpio0a0~gpio0b3
#define RK3588_PMU1_IOC_GPIO0A_IOMUX_SEL_H      (RK3588_PMU1_IOC_BASE + 0x04U)    //gpio0a4~gpio0b7
#define RK3588_PMU1_IOC_GPIO0B_IOMUX_SEL_L      (RK3588_PMU1_IOC_BASE + 0x08U)    //gpio0b0~gpio0b3
#define RK3588_PMU1_IOC_GPIO0A_P                (RK3588_PMU1_IOC_BASE + 0x20U)
#define RK3588_PMU1_IOC_GPIO0B_P                (RK3588_PMU1_IOC_BASE + 0x24U)

#define RK3588_PMU2_IOC_BASE                    0xfd5f4000U
#define RK3588_PMU2_IOC_GPIO0B_IOMUX_SEL_H      (RK3588_PMU2_IOC_BASE + 0x00U)    //gpio0a5~gpio0b7
#define RK3588_PMU2_IOC_GPIO0C_IOMUX_SEL_L      (RK3588_PMU2_IOC_BASE + 0x04U)    //gpio0a0~gpio0b3
#define RK3588_PMU2_IOC_GPIO0C_IOMUX_SEL_H      (RK3588_PMU2_IOC_BASE + 0x08U)    //gpio0a4~gpio0b7
#define RK3588_PMU2_IOC_GPIO0D_IOMUX_SEL_L      (RK3588_PMU2_IOC_BASE + 0x0cU)    //gpio0a0~gpio0b3
#define RK3588_PMU2_IOC_GPIO0D_IOMUX_SEL_H      (RK3588_PMU2_IOC_BASE + 0x10U)    //gpio0a4~gpio0b6
#define RK3588_PMU2_IOC_GPIO0B_P                (RK3588_PMU2_IOC_BASE + 0x28U)
#define RK3588_PMU2_IOC_GPIO0C_P                (RK3588_PMU2_IOC_BASE + 0x2cU)
#define RK3588_PMU2_IOC_GPIO0D_P                (RK3588_PMU2_IOC_BASE + 0x30U)

#define RK3588_BUS_IOC_BASE                     0xfd5f8000U

//gpio pull up/down
#define RK3588_VCCIO1_4_IOC_BASE                0xfd5f9000U
#define RK3588_VCCIO1_4_IOC_GPIO1A_P            (RK3588_VCCIO1_4_IOC_BASE + 0x0110U)

#define RK3588_VCCIO3_5_IOC_BASE                0xfd5fa000U
#define RK3588_VCCIO3_5_IOC_GPIO2A_P            (RK3588_VCCIO3_5_IOC_BASE + 0x0120U)

#define RK3588_VCCIO6_IOC_BASE                  0xfd5fc000U
#define RK3588_VCCIO6_IOC_GPIO4A_P              (RK3588_VCCIO6_IOC_BASE + 0x0140U)

//pwm register base addr
#define RK3588_PWM0_BASE                        0xfd8b0000U
#define RK3588_PWM1_BASE                        0xfe8d0000U
#define RK3588_PWM2_BASE                        0xfebe0000U
#define RK3588_PWM3_BASE                        0xfebf0000U

//CH0
#define RK3588_CH0_PERIOD_HPR(base)             (base + 0x04U)
#define RK3588_CH0_DUTY_LPR(base)               (base + 0x08U)
#define RK3588_CH0_CTRL(base)                   (base + 0x0CU)

//CH1
#define RK3588_CH1_PERIOD_HPR(base)             (base + 0x14U)
#define RK3588_CH1_DUTY_LPR(base)               (base + 0x18U)
#define RK3588_CH1_CTRL(base)                   (base + 0x1CU)

//CH2
#define RK3588_CH2_PERIOD_HPR(base)             (base + 0x24U)
#define RK3588_CH2_DUTY_LPR(base)               (base + 0x28U)
#define RK3588_CH2_CTRL(base)                   (base + 0x2CU)

//CH3
#define RK3588_CH3_PERIOD_HPR(base)             (base + 0x34U)
#define RK3588_CH3_DUTY_LPR(base)               (base + 0x38U)
#define RK3588_CH3_CTRL(base)                   (base + 0x3CU)

//pwm_ctrl_register
#define PWM_CTRL_RPT                            (24)    // 24 ~ 31
#define PWM_CTRL_SCALE                          (16)    // 16 ~ 23
#define PWM_CTRL_PRESCALE                       (12)    // 12 ~ 14
#define PWM_CTRL_CLK_SRC_SEL                    (10)
#define PWM_CTRL_CLK_SEL                        (9)
#define PWM_CTRL_FORCE_CLK_EN                   (8)
#define PWM_CTRL_CH_CNT_EN                      (7)
#define PWM_CTRL_CONLOCK                        (6)
#define PWM_CTRL_OUTPUT_MODE                    (5)
#define PWM_CTRL_INACTIVE_POL                   (4)
#define PWM_CTRL_DUTY_POL                       (3)
#define PWM_CTRL_PWM_MODE                       (1)     // 1 ~ 2
#define PWM_CTRL_PWM_EN                         (0)

#define PWM_CTRL_SCALE_MAX                      512

typedef int (pin_group_t)[32]; 

typedef struct {
	pin_group_t *pin_mask;

	void *gpio0_base;
	void *gpio1_base;
	void *gpio2_base;
	void *gpio3_base;
	void *gpio4_base;

	void *pmu1_ioc_base;
	void *pmu2_ioc_base;
	void *bus_ioc_base;

	void *cru_base;
	void *pmu1cru_base;

	void *vccio1_4_ioc_base;
	void *vccio3_5_ioc_base;
	void *vccio6_ioc_base;

	void *pwm0_base;
	void *pwm1_base;
	void *pwm2_base;
	void *pwm3_base;

} rk3588_soc_t;

extern int rk3588_setupReg(int fd);
extern int rk3588_setPinMask(int model);
extern int rk3588_getPinMode(int pin);
extern int rk3588_setPinMode(int pin, int mode);
extern int rk3588_setPullUpDn(int pin, int pud);
extern int rk3588_digitalRead(int pin);
extern int rk3588_digitalWrite(int pin, int value);
extern int rk3588_pwmWrite(int pin, unsigned int value);
extern int rk3588_pwmClock(int pin, unsigned int divisor);
extern int rk3588_pwmRange(int pin, unsigned int range);

#ifdef __cplusplus
}
#endif

#endif
