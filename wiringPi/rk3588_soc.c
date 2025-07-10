#include <stdio.h>
#include <unistd.h>
#include "wiringPi.h"
#include "rk3588_soc.h"

rk3588_soc_t rk3588_soc;

static int rk3588_pin_mask_5A[5][32] =  //[BANK]	[INDEX]
{
	{ 0, 1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO0
	{-1,-1, 2, 3, 4,-1, 6, 7,  0, 1, 2, 3, 4, 5, 6, 7, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1, 6, 7,},//GPIO1
	{-1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO2
	{-1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1, 4,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO3
	{ 0, 1, 2, 3, 4, 5,-1,-1,  0, 1, 2, 3,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO4
};

static int (*rk3588_pin_mask)[32] = NULL;

static void rk3588_set_pwm_reg(int pin, rk3588_soc_t *soc)
{
	soc->pwm_mux = RK3588_BUS_IOC_BASE + ((pin >> 2) << 2);
	soc->pwm_mux_val = 0xb;
	soc->pwm_mux_offset = (pin % 4) << 2;

	if (pin == 15 || pin == 16 || pin == 28) {
		soc->pwm_mux = RK3588_PMU2_IOC_BASE + (((pin >> 2) - 3) << 2);
		soc->pwm_mux_val = 0x3;
	}

	switch (pin) {
		case 15:
		case 34:
		case 58:  //PWM0CH0
			soc->pwm_base = RK3588_PWM0_BASE;
			soc->ch_period_hpr = RK3588_CH0_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH0_DUTY_LPR;
			soc->ch_crtl = RK3588_CH0_CTRL;
			break;
		case 16:
		case 35:
		case 59:  //PWM0CH1
			soc->pwm_base = RK3588_PWM0_BASE;
			soc->ch_period_hpr = RK3588_CH1_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH1_DUTY_LPR;
			soc->ch_crtl = RK3588_CH1_CTRL;
			break;
		case 39:
		case 28:
		case 50:  //PWM0CH3
			soc->pwm_base = RK3588_PWM0_BASE;
			soc->ch_period_hpr = RK3588_CH3_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH3_DUTY_LPR;
			soc->ch_crtl = RK3588_CH3_CTRL;
			break;
		case 96:  //PWM2CH2
			soc->pwm_base = RK3588_PWM2_BASE;
			soc->ch_period_hpr = RK3588_CH2_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH2_DUTY_LPR;
			soc->ch_crtl = RK3588_CH2_CTRL;
			break;
		case 97:  //PWM2CH3
			soc->pwm_base = RK3588_PWM2_BASE;
			soc->ch_period_hpr = RK3588_CH3_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH3_DUTY_LPR;
			soc->ch_crtl = RK3588_CH3_CTRL;
			break;
		case 109:  //PWM3CH0
			soc->pwm_base = RK3588_PWM3_BASE;
			soc->ch_period_hpr = RK3588_CH0_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH0_DUTY_LPR;
			soc->ch_crtl = RK3588_CH0_CTRL;
			break;
		case 47:
		case 110:  //PWM3CH1
			soc->pwm_base = RK3588_PWM3_BASE;
			soc->ch_period_hpr = RK3588_CH1_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH1_DUTY_LPR;
			soc->ch_crtl = RK3588_CH1_CTRL;
			break;
		case 62:
		case 114:
		case 138:  //PWM3CH2
			soc->pwm_base = RK3588_PWM3_BASE;
			soc->ch_period_hpr = RK3588_CH2_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH2_DUTY_LPR;
			soc->ch_crtl = RK3588_CH2_CTRL;
			break;
		case 54:
		case 63:
		case 139:  //PWM3CH3
			soc->pwm_base = RK3588_PWM3_BASE;
			soc->ch_period_hpr = RK3588_CH3_PERIOD_HPR;
			soc->ch_duty_lpr = RK3588_CH3_DUTY_LPR;
			soc->ch_crtl = RK3588_CH3_CTRL;
			break;
		default:
			break;
	}
}

/*
 * Set pin mask
 */
void rk3588_setPinMask(int model)
{
	if (model == PI_MODEL_HAIBOX_5A) {
		rk3588_pin_mask = rk3588_pin_mask_5A;
	} else {
		rk3588_pin_mask = NULL;
	}
}

/*
 * Read register value helper  
 */
uint32_t rk3588_readR(uint32_t addr)
{
	uint32_t reg_val = 0;
	uint32_t mmap_base;
	uint32_t mmap_seek;

	mmap_base = (addr & (0xfffff000));
	mmap_seek = (addr - mmap_base);

	if(mmap_base == RK3588_GPIO0_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio0_base + mmap_seek));
	else if(mmap_base == RK3588_GPIO1_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio1_base + mmap_seek));
	else if(mmap_base == RK3588_GPIO2_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio2_base + mmap_seek));
	else if(mmap_base == RK3588_GPIO3_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio3_base + mmap_seek));
	else if(mmap_base == RK3588_GPIO4_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio4_base + mmap_seek));
	else if(mmap_base == RK3588_PMU1_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pmu1_ioc_base + mmap_seek));
	else if(mmap_base == RK3588_BUS_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.bus_ioc_base + mmap_seek));
	else if(mmap_base == RK3588_CRU_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.cur_base + mmap_seek));
	else if(mmap_base == RK3588_PMU1CRU_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pmu1cur_base + mmap_seek));
	else if(mmap_base == RK3588_PMU2_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pmu2_ioc_base + mmap_seek));
	else if(mmap_base == RK3588_VCCIO1_4_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.vccio1_4_ioc_base + mmap_seek));
	else if(mmap_base == RK3588_VCCIO3_5_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.vccio3_5_ioc_base + mmap_seek));
	else if(mmap_base == RK3588_VCCIO6_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.vccio6_ioc_base + mmap_seek));
	else if(mmap_base == RK3588_PWM0_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm0_base + mmap_seek));
	else if(mmap_base == RK3588_PWM1_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm1_base + mmap_seek));
	else if(mmap_base == RK3588_PWM2_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm2_base + mmap_seek));
	else if(mmap_base == RK3588_PWM3_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm3_base + mmap_seek));

	return reg_val;
}

/*
 * Wirte value to register helper
 */
void rk3588_writeR(uint32_t addr, uint32_t val)
{
	uint32_t mmap_base;
	uint32_t mmap_seek;

	mmap_base = (addr & (~0xfff));
	mmap_seek = (addr - mmap_base);

	if(mmap_base == RK3588_GPIO0_BASE)
		*((uint32_t*)(rk3588_soc.gpio0_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_GPIO1_BASE)
		*((uint32_t*)(rk3588_soc.gpio1_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_GPIO2_BASE)
		*((uint32_t*)(rk3588_soc.gpio2_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_GPIO3_BASE)
		*((uint32_t*)(rk3588_soc.gpio3_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_GPIO4_BASE)
		*((uint32_t*)(rk3588_soc.gpio4_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PMU1_IOC_BASE)
		*((uint32_t*)(rk3588_soc.pmu1_ioc_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_BUS_IOC_BASE)
		*((uint32_t*)(rk3588_soc.bus_ioc_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_CRU_BASE)
		*((uint32_t*)(rk3588_soc.cur_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PMU1CRU_BASE)
		*((uint32_t*)(rk3588_soc.pmu1cur_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PMU2_IOC_BASE)
		*((uint32_t*)(rk3588_soc.pmu2_ioc_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_VCCIO1_4_IOC_BASE)
		*((uint32_t*)(rk3588_soc.vccio1_4_ioc_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_VCCIO3_5_IOC_BASE)
		*((uint32_t*)(rk3588_soc.vccio3_5_ioc_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_VCCIO6_IOC_BASE)
		*((uint32_t*)(rk3588_soc.vccio6_ioc_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PWM0_BASE)
		*((uint32_t*)(rk3588_soc.pwm0_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PWM1_BASE)
		*((uint32_t*)(rk3588_soc.pwm1_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PWM2_BASE)
		*((uint32_t*)(rk3588_soc.pwm2_base + mmap_seek)) = val;
	else if(mmap_base == RK3588_PWM3_BASE)
		*((uint32_t*)(rk3588_soc.pwm3_base + mmap_seek)) = val;

}

/*
 * Read digital pin
 */
int rk3588_digitalRead(int pin)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = 0;

	if (!rk3588_pin_mask || rk3588_pin_mask[bank][index] == -1)
		return 0;

	if (bank == 0)
		phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_EXT_PORT_OFFSET;
	else if (bank == 1)
		phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_EXT_PORT_OFFSET;
	else if (bank == 2)
		phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_EXT_PORT_OFFSET;
	else if (bank == 3)
		phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_EXT_PORT_OFFSET;
	else if (bank == 4)
		phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_EXT_PORT_OFFSET;
	else
		return 0;

	return (rk3588_readR(phyaddr) >> index) & 0x01;
}

/*
 * Write digital pin
 */
int rk3588_digitalWrite(int pin, int value)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t dr_phyaddr;
	uint32_t cru_phyaddr;
	uint32_t cru_val;
	uint32_t reg_val;

	if (!rk3588_pin_mask || rk3588_pin_mask[bank][index] == -1)
		return 0;

	if (bank == 0) {
		dr_phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_PMU1CRU_BASE + RK3588_PMU1CRU_GATE_CON5_OFFSET;
		cru_val = 0xffff9fff;
	}
	else if (bank == 1) {
		dr_phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON16_OFFSET;
		cru_val = 0xffff3fff;
	}
	else if (bank == 2) {
		dr_phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON17_OFFSET;
		cru_val = 0xffffffc0;
	}
	else if(bank == 3){
		dr_phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON17_OFFSET;
		cru_val = 0xffffffc0;
	}
	else if (bank == 4) {
		dr_phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON17_OFFSET;
		cru_val = 0xffffffc0;
	}
	else {
		return 0;
	}

	reg_val = rk3588_readR(cru_phyaddr);
	reg_val &= cru_val;
	rk3588_writeR(cru_phyaddr, reg_val);

	if (!value) {
		reg_val = rk3588_readR(dr_phyaddr);
		reg_val |= 0xffff0000;
		reg_val &= ~(1 << (index % 16));
		rk3588_writeR(dr_phyaddr, reg_val);
		reg_val = rk3588_readR(dr_phyaddr);
	} else {
		reg_val = rk3588_readR(dr_phyaddr);
		reg_val |= 0xffff0000;
		reg_val |= (1 << (index % 16));
		rk3588_writeR(dr_phyaddr, reg_val);
		reg_val = rk3588_readR(dr_phyaddr);
	}

	return 0;
}

/*
 * Set output PWM value
 */
int rk3588_pwmWrite(int pin, int value)
{
	uint32_t reg_val;

	rk3588_set_pwm_reg(pin, &rk3588_soc);

	reg_val = rk3588_readR(RK3588_CH_PERIOD_HPR);

	if ((uint32_t)value > reg_val) {
		fprintf (stderr, "gpio: CCR should be less than or equal to ARR (%d)\n", reg_val);
		return -1;
	}

	//enable conlock
	reg_val = rk3588_readR(RK3588_CH_CTRL);
	reg_val |= (1 << RK3588_CONLOCK);
	rk3588_writeR(RK3588_CH_CTRL, reg_val);
	delay(1);
	reg_val = rk3588_readR(RK3588_CH_CTRL);

	//modifine act
	rk3588_writeR(RK3588_CH_DUTY_LPR, value);
	delay(1);
	reg_val = rk3588_readR(RK3588_CH_DUTY_LPR);

	//disable conlock
	reg_val = rk3588_readR(RK3588_CH_CTRL);
	reg_val &= ~(1 << RK3588_CONLOCK);
	rk3588_writeR(RK3588_CH_CTRL, reg_val);
	delay(1);
	reg_val = rk3588_readR(RK3588_CH_CTRL);

	return 0;
}

/*
 * Returns the ALT bits for a given port
 */
int rk3588_getAlt(int pin)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t ddr_phyaddr;
	uint32_t pmu1_ioc_phyaddr;
	uint32_t bus_ioc_phyaddr;
	uint32_t reg_val;
	int alt_mode;

	if (!rk3588_pin_mask || rk3588_pin_mask[bank][index] == -1)
		return 0;

	bus_ioc_phyaddr = RK3588_BUS_IOC_BASE + (0x20 * bank) + ((index >> 2) << 2);
	if (bank == 0) {
		if (index >= 12)
			pmu1_ioc_phyaddr = RK3588_PMU2_IOC_BASE + ((index - 12) >> 2 << 2);
		else
			pmu1_ioc_phyaddr = RK3588_PMU1_IOC_BASE + (index >> 2 << 2);
		ddr_phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
	}
	else if (bank == 1) {
		ddr_phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
	}
	else if (bank == 2) {
		ddr_phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
	}
	else if (bank == 3) {
		ddr_phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
	}
	else if (bank == 4) {
		ddr_phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
	} else {
		return 0;
	}

	if (bank == 0) {
		reg_val = rk3588_readR(pmu1_ioc_phyaddr);
		alt_mode = (reg_val >> ((index % 4) << 2)) & 0xf;
		if (alt_mode == 0x8)
			reg_val = rk3588_readR(bus_ioc_phyaddr);
	}
	else {
		reg_val = rk3588_readR(bus_ioc_phyaddr);
	}

	alt_mode = (reg_val >> ((index % 4) << 2)) & 0xf;//获取控制模式的那四位的值
	if (alt_mode == 0) { //如果是gpio模式
		reg_val = rk3588_readR(ddr_phyaddr);	//获取gpio方向寄存器的值
		return (reg_val >> (index % 16)) & 0x1;	//index对应的gpio的方向值，0为in，1为out
	}

	return alt_mode + 1; //如果不是gpio模式，返回的alt，从2开始，0和1是in和out
}

/*
 * Set pin mode
 */
int rk3588_pinMode(int pin, int mode)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t ddr_phyaddr;
	uint32_t cru_phyaddr;
	uint32_t bus_ioc_phyaddr;
	uint32_t cru_val;
	uint32_t reg_val;

	if (!rk3588_pin_mask || rk3588_pin_mask[bank][index] == -1)
		return 0;

	bus_ioc_phyaddr = RK3588_BUS_IOC_BASE + (0x20 * bank) + ((index >> 2) << 2);
	if (bank == 0) {
		if (index >= 12)
			bus_ioc_phyaddr = RK3588_PMU2_IOC_BASE + ((index - 12) >> 2 << 2);
		else
			bus_ioc_phyaddr = RK3588_PMU1_IOC_BASE + (index >> 2 << 2);
		ddr_phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_PMU1CRU_BASE + RK3588_PMU1CRU_GATE_CON5_OFFSET;
		cru_val = 0xffff9fff;
	}
	else if (bank == 1) {
		ddr_phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON16_OFFSET;
		cru_val = 0xffff3fff;
	}
	else if (bank == 2) {
		ddr_phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON17_OFFSET;
		cru_val = 0xffffffc0;
	}
	else if (bank == 3) {
		ddr_phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON17_OFFSET;
		cru_val = 0xffffffc0;
	}
	else if (bank == 4) {
		ddr_phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index / 16) << 2);
		cru_phyaddr = RK3588_CRU_BASE + RK3588_CRU_GATE_CON17_OFFSET;
		cru_val = 0xffffffc0;
	}
	else {
		return 0;
	}

	if (mode == INPUT) {
		reg_val = rk3588_readR(cru_phyaddr);
		reg_val |= 0xffff0000;
		reg_val &= cru_val;
		rk3588_writeR(cru_phyaddr, reg_val);
		reg_val = rk3588_readR(cru_phyaddr);

		reg_val = rk3588_readR(bus_ioc_phyaddr);
		reg_val |= 0xffff0000;
		reg_val &= ~(0xf << ((index % 4) << 2));
		rk3588_writeR(bus_ioc_phyaddr, reg_val);
		reg_val = rk3588_readR(bus_ioc_phyaddr);

		reg_val = rk3588_readR(ddr_phyaddr);
		reg_val |= 0xffff0000;
		reg_val &= ~(1 << (index % 16));
		rk3588_writeR(ddr_phyaddr, reg_val);
	} 
	else if (mode == OUTPUT) {
		reg_val = rk3588_readR(cru_phyaddr);
		reg_val |= 0xffff0000;
		reg_val &= cru_val;
		rk3588_writeR(cru_phyaddr, reg_val);
		reg_val = rk3588_readR(cru_phyaddr);

		reg_val = rk3588_readR(bus_ioc_phyaddr);
		reg_val |= 0xffff0000;
		reg_val &= ~(0xf << ((index % 4) << 2));
		rk3588_writeR(bus_ioc_phyaddr, reg_val);
		reg_val = rk3588_readR(bus_ioc_phyaddr);

		reg_val = rk3588_readR(ddr_phyaddr);
		reg_val |= 0xffff0000;
		reg_val |= (1 << (index % 16));
		rk3588_writeR(ddr_phyaddr, reg_val);
	}
	else if (mode == PWM_OUTPUT) {
		//set clk——busioc
		reg_val = rk3588_readR(RK3588_CRU_GATE_CON19);
		reg_val &= 0xfffffffe;
		rk3588_writeR(RK3588_CRU_GATE_CON19, reg_val);
		reg_val = rk3588_readR(RK3588_CRU_GATE_CON19);

		//Set clk——pwm123
		reg_val = rk3588_readR(RK3588_CRU_GATE_CON15);
		reg_val &= ~(0x00000ff8);
		reg_val |= 0xffff0920;
		rk3588_writeR(RK3588_CRU_GATE_CON15, reg_val);
		reg_val = rk3588_readR(RK3588_CRU_GATE_CON15);

		//Set clk——pmu1pwm
		reg_val = rk3588_readR(RK3588_PMU1CRU_GATE_CON1);
		reg_val &= ~(0x00007000);
		reg_val |= 0xffff4000;
		rk3588_writeR(RK3588_PMU1CRU_GATE_CON1, reg_val);
		reg_val = rk3588_readR(RK3588_PMU1CRU_GATE_CON1);

		rk3588_set_pwm_reg(pin, &rk3588_soc);

		//Set mux
		reg_val = rk3588_readR(RK3588_PWM_MUX);
		reg_val |= 0xffff0000;
		reg_val &= ~(0xf << RK3588_PWM_MUX_OFFSET);
		reg_val |= (RK3588_PWM_MUX_VAL << RK3588_PWM_MUX_OFFSET);
		rk3588_writeR(RK3588_PWM_MUX, reg_val);
		reg_val = rk3588_readR(RK3588_PWM_MUX);

		//clear all reg
		rk3588_writeR(RK3588_CH_PERIOD_HPR, 0);
		reg_val = rk3588_readR(RK3588_CH_PERIOD_HPR);

		rk3588_writeR(RK3588_CH_DUTY_LPR, 0);
		reg_val = rk3588_readR(RK3588_CH_DUTY_LPR);

		rk3588_writeR(RK3588_CH_CTRL, 0);
		reg_val = rk3588_readR(RK3588_CH_CTRL);

		//Set period
		reg_val = rk3588_readR(RK3588_CH_PERIOD_HPR);
		reg_val = 1000;
		rk3588_writeR(RK3588_CH_PERIOD_HPR, reg_val);
		reg_val = rk3588_readR(RK3588_CH_PERIOD_HPR);

		//Set duty
		reg_val = rk3588_readR(RK3588_CH_DUTY_LPR);
		reg_val = 500;
		rk3588_writeR(RK3588_CH_DUTY_LPR, reg_val);
		reg_val = rk3588_readR(RK3588_CH_DUTY_LPR);

		//Set ctrl
		/**
		* frequency of clock: 24 MHz / 120 = 200 kHz
		* frequency of PWM: 200 kHz / 1000 = 200 Hz
		* period of PWM: 1 / 200 Hz = 0.005 s
		* duty: 500/1000 = 50%
		*/
		reg_val = rk3588_readR(RK3588_CH_CTRL);
		reg_val = (0 << RK3588_RPT) \
				| (60 << RK3588_SCALE) \
				| (0 << RK3588_PRESCALE) \
				| (0 << RK3588_CLK_SRC_SEL) \
				| (1 << RK3588_CLK_SEL) \
				| (1 << RK3588_FORCE_CLK_EN) \
				| (1 << RK3588_CH_CNT_EN) \
				| (0 << RK3588_CONLOCK) \
				| (0 << RK3588_OUTPUT_MODE) \
				| (0 << RK3588_INACTIVE_POL) \
				| (1 << RK3588_DUTY_POL) \
				| (1 << RK3588_PWM_MODE) \
				| (1 << RK3588_PWM_EN);

		rk3588_writeR(RK3588_CH_CTRL, reg_val);
		reg_val = rk3588_readR(RK3588_CH_CTRL);
	}

	return 0;
}

/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin.
 */
int rk3588_pullUpDnControl(int pin, int pud)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr;
	uint32_t offset;
	uint32_t bit_enable;
	uint32_t bit_value;
	uint32_t reg_val;

	if (!rk3588_pin_mask || rk3588_pin_mask[bank][index] == -1)
		return 0;

	if(bank == 0 && index < 12)
		phyaddr = RK3588_PMU1_IOC_BASE + RK3588_PMU1_IOC_GPIO0A_P + ((index >> 3) << 2);
	else if(bank == 0 && index > 11 && index < 31)
		phyaddr = RK3588_PMU2_IOC_BASE + RK3588_PMU2_IOC_GPIO0B_P + (((index - 8) >> 3) << 2);
	else if(bank == 1)
		phyaddr = RK3588_VCCIO1_4_IOC_BASE + RK3588_VCCIO1_4_IOC_GPIO1A_P + ((index >> 3) << 2);
	else if(bank < 4 || (bank == 4 && index > 17))
		phyaddr = RK3588_VCCIO3_5_IOC_BASE + RK3588_VCCIO3_5_IOC_GPIO2A_P + (((pin - 64) >> 3) << 2);
	else if(bank == 4 && index < 18)
		phyaddr = RK3588_VCCIO6_IOC_BASE + RK3588_VCCIO6_IOC_GPIO4A_P + ((index >> 3) << 2);
	else
		return 0;

	offset = (index % 8) << 1;
	bit_enable = 3 << ( 16 + offset);

	/* */if (pud == PUD_UP)
		bit_value = 3;
	else if (pud == PUD_DOWN)
		bit_value = 1;
	else
		bit_value = 0;

	reg_val = rk3588_readR(phyaddr);

	/* clear bit */
	reg_val &= ~(3 << offset);

	/* bit write enable*/
	reg_val |= bit_enable;

	/* set bit */
	reg_val |= (bit_value & 3) << offset;

	rk3588_writeR(phyaddr, reg_val);
	reg_val = rk3588_readR(phyaddr);

	return 0;
}
