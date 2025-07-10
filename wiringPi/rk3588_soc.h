#ifndef	__RK3588_SOC_H__
#define	__RK3588_SOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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
#define RK3588_CRU_GATE_CON16_OFFSET            0x0840U    //for gpio1 bit 14 15    30 31
#define RK3588_CRU_GATE_CON17_OFFSET            0x0844U    //for gpio2/3/4 - bit 0 1 2 3 4 5    16 17 18 19 20 21

#define RK3588_PMU1CRU_BASE                     0xfd7f0000U
#define RK3588_PMU1CRU_GATE_CON5_OFFSET         0x0814U    //for gpio0 - bit 5 6    21 22

#define RK3588_GPIO_NUM                         (0x40)
#define RK3588_GPIO_BIT(x)                      (1UL << (x))

//gpio iomux
#define RK3588_PMU1_IOC_BASE                    0xfd5f0000U
#define RK3588_PMU1_IOC_GPIO0A_IOMUX_SEL_L      0x00U    //gpio0a0~gpio0b3
#define RK3588_PMU1_IOC_GPIO0A_IOMUX_SEL_H      0x04U    //gpio0a4~gpio0b7
#define RK3588_PMU1_IOC_GPIO0B_IOMUX_SEL_L      0x08U    //gpio0b0~gpio0b3

#define RK3588_PMU2_IOC_BASE                    0xfd5f4000U
#define RK3588_PMU2_IOC_GPIO0B_IOMUX_SEL_H      0x00U    //gpio0a5~gpio0b7
#define RK3588_PMU2_IOC_GPIO0C_IOMUX_SEL_L      0x04U    //gpio0a0~gpio0b3
#define RK3588_PMU2_IOC_GPIO0C_IOMUX_SEL_H      0x08U    //gpio0a4~gpio0b7
#define RK3588_PMU2_IOC_GPIO0D_IOMUX_SEL_L      0x0cU    //gpio0a0~gpio0b3
#define RK3588_PMU2_IOC_GPIO0D_IOMUX_SEL_H      0x10U    //gpio0a4~gpio0b6

#define RK3588_BUS_IOC_BASE                     0xfd5f8000U

//gpio pull up/down
#define RK3588_VCCIO1_4_IOC_BASE                0xfd5f9000U
#define RK3588_VCCIO3_5_IOC_BASE                0xfd5fa000U
#define RK3588_VCCIO6_IOC_BASE                  0xfd5fc000U

#define RK3588_PMU1_IOC_GPIO0A_P                0x0020U
#define RK3588_PMU1_IOC_GPIO0B_P                0x0024U
#define RK3588_PMU2_IOC_GPIO0B_P                0x0028U
#define RK3588_PMU2_IOC_GPIO0C_P                0x002cU
#define RK3588_PMU2_IOC_GPIO0D_P                0x0030U
#define RK3588_VCCIO1_4_IOC_GPIO1A_P            0x0110U
#define RK3588_VCCIO3_5_IOC_GPIO2A_P            0x0120U
#define RK3588_VCCIO6_IOC_GPIO4A_P              0x0140U

//pwm register base addr
//#define RK3588_CRU_BASE                       0xfd7c0000U
//#define RK3588_PMU1CRU_BASE                   0xfd7f0000U
#define RK3588_PWM0_BASE                        0xfd8b0000U
#define RK3588_PWM1_BASE                        0xfe8d0000U
#define RK3588_PWM2_BASE                        0xfebe0000U
#define RK3588_PWM3_BASE                        0xfebf0000U

//cru
#define RK3588_CRU_GATE_CON19                   (RK3588_CRU_BASE + 0x084CU)    //for busioc_clk_en
#define RK3588_CRU_GATE_CON15                   (RK3588_CRU_BASE + 0x083CU)    //for pwm123_clk_en
#define RK3588_PMU1CRU_GATE_CON1                (RK3588_PMU1CRU_BASE +0x0804U)    //for pmu1pwm_clk_en

//CH0
#define RK3588_CH0_PERIOD_HPR                   (RK3588_PWM_BASE + 0x04)
#define RK3588_CH0_DUTY_LPR                     (RK3588_PWM_BASE + 0x08)
#define RK3588_CH0_CTRL                         (RK3588_PWM_BASE + 0x0C)

//CH1
#define RK3588_CH1_PERIOD_HPR                   (RK3588_PWM_BASE + 0x14)
#define RK3588_CH1_DUTY_LPR                     (RK3588_PWM_BASE + 0x18)
#define RK3588_CH1_CTRL                         (RK3588_PWM_BASE + 0x1C)

//CH2
#define RK3588_CH2_PERIOD_HPR                   (RK3588_PWM_BASE + 0x24)
#define RK3588_CH2_DUTY_LPR                     (RK3588_PWM_BASE + 0x28)
#define RK3588_CH2_CTRL                         (RK3588_PWM_BASE + 0x2C)

//CH3
#define RK3588_CH3_PERIOD_HPR                   (RK3588_PWM_BASE + 0x34)
#define RK3588_CH3_DUTY_LPR                     (RK3588_PWM_BASE + 0x38)
#define RK3588_CH3_CTRL                         (RK3588_PWM_BASE + 0x3C)

//for short——pwm
#define RK3588_PWM_BASE                         (rk3588_soc.pwm_base)
#define RK3588_PWM_MUX                          (rk3588_soc.pwm_mux)
#define RK3588_PWM_MUX_VAL                      (rk3588_soc.pwm_mux_val)
#define RK3588_PWM_MUX_OFFSET                   (rk3588_soc.pwm_mux_offset)
#define RK3588_CH_PERIOD_HPR                    (rk3588_soc.ch_period_hpr)
#define RK3588_CH_DUTY_LPR                      (rk3588_soc.ch_duty_lpr)
#define RK3588_CH_CTRL                          (rk3588_soc.ch_crtl)

//pwm_ctrl_offset
#define RK3588_RPT                              (24)    // 24 ~ 31
#define RK3588_SCALE                            (16)    // 16 ~ 23
#define RK3588_PRESCALE                         (12)    // 12 ~ 14
#define RK3588_CLK_SRC_SEL                      (10)
#define RK3588_CLK_SEL                          (9)
#define RK3588_FORCE_CLK_EN                     (8)
#define RK3588_CH_CNT_EN                        (7)
#define RK3588_CONLOCK                          (6)
#define RK3588_OUTPUT_MODE                      (5)
#define RK3588_INACTIVE_POL                     (4)
#define RK3588_DUTY_POL                         (3)
#define RK3588_PWM_MODE                         (1)     // 1 ~ 2
#define RK3588_PWM_EN                           (0)

typedef struct {
	void *gpio0_base;
	void *gpio1_base;
	void *gpio2_base;
	void *gpio3_base;
	void *gpio4_base;

	void *pmu1_ioc_base;
	void *pmu2_ioc_base;
	void *bus_ioc_base;

	void *cur_base;
	void *pmu1cur_base;

	void *vccio1_4_ioc_base;
	void *vccio3_5_ioc_base;
	void *vccio6_ioc_base;

	void *pwm0_base;
	void *pwm1_base;
	void *pwm2_base;
	void *pwm3_base;
	uint32_t pwm_base;
	uint32_t pwm_mux;
	uint32_t pwm_mux_val;
	uint32_t pwm_mux_offset;
	uint32_t ch_period_hpr;
	uint32_t ch_duty_lpr;
	uint32_t ch_crtl;

} rk3588_soc_t;

extern rk3588_soc_t rk3588_soc;

extern void rk3588_setPinMask(int model);
extern uint32_t rk3588_readR(uint32_t addr);
extern void rk3588_writeR(uint32_t addr, uint32_t val);
extern int rk3588_digitalRead(int pin);
extern int rk3588_digitalWrite(int pin, int value);
extern int rk3588_pwmWrite(int pin, int value);
extern int rk3588_getAlt(int pin);
extern int rk3588_pinMode(int pin, int mode);
extern int rk3588_pullUpDnControl(int pin, int pud);

#ifdef __cplusplus
}
#endif

#endif
