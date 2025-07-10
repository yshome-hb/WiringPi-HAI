#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>

#include "wiringPi.h"
#include "rk3588_soc.h"

#define	BLOCK_SIZE (4 * 1024)

#define RK_DELAY(ms) \
	do { \
  		struct timespec sleeper; \
 	 	sleeper.tv_sec = (time_t)(ms / 1000); \
  		sleeper.tv_nsec = (long)(ms % 1000) * 1000000; \
  		nanosleep(&sleeper, NULL); \
	} while (0)

typedef struct {
	uint32_t pwm_base;
	uint32_t pwm_mux_addr;
	uint32_t pwm_mux_value;
	uint32_t pwm_mux_offset;
	uint32_t ch_period_hpr;
	uint32_t ch_duty_lpr;
	uint32_t ch_crtl;
} pwm_config_t;

static const pin_group_t haibox_5a_pin_mask[5] = { //[BANK][INDEX]
	{-1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO0
	{ 0, 1, 2, 3, 4,-1, 6, 7,  0, 1, 2, 3, 4, 5, 6, 7, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1, 6, 7,},//GPIO1
	{-1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO2
	{-1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1, 4,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO3
	{ 0, 1, 2, 3, 4, 5,-1,-1,  0, 1, 2, 3,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1, -1,-1,-1,-1,-1,-1,-1,-1,},//GPIO4
};

static rk3588_soc_t rk3588_soc = {
	.pin_mask = NULL,
};

/*
 * Read register value helper  
 */
uint32_t rk3588_read_reg(uint32_t addr)
{
	uint32_t reg_val = 0;
	uint32_t mmap_base;
	uint32_t mmap_seek;

	mmap_base = (addr & (0xfffff000));
	mmap_seek = (addr - mmap_base);

	if (mmap_base == RK3588_GPIO0_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio0_base + mmap_seek));
	else if (mmap_base == RK3588_GPIO1_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio1_base + mmap_seek));
	else if (mmap_base == RK3588_GPIO2_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio2_base + mmap_seek));
	else if (mmap_base == RK3588_GPIO3_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio3_base + mmap_seek));
	else if (mmap_base == RK3588_GPIO4_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.gpio4_base + mmap_seek));
	else if (mmap_base == RK3588_PMU1_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pmu1_ioc_base + mmap_seek));
	else if (mmap_base == RK3588_PMU2_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pmu2_ioc_base + mmap_seek));
	else if (mmap_base == RK3588_BUS_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.bus_ioc_base + mmap_seek));
	else if (mmap_base == RK3588_CRU_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.cru_base + mmap_seek));
	else if (mmap_base == RK3588_PMU1CRU_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pmu1cru_base + mmap_seek));
	else if (mmap_base == RK3588_VCCIO1_4_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.vccio1_4_ioc_base + mmap_seek));
	else if (mmap_base == RK3588_VCCIO3_5_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.vccio3_5_ioc_base + mmap_seek));
	else if (mmap_base == RK3588_VCCIO6_IOC_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.vccio6_ioc_base + mmap_seek));
	else if (mmap_base == RK3588_PWM0_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm0_base + mmap_seek));
	else if (mmap_base == RK3588_PWM1_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm1_base + mmap_seek));
	else if (mmap_base == RK3588_PWM2_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm2_base + mmap_seek));
	else if (mmap_base == RK3588_PWM3_BASE)
		reg_val = *((uint32_t*)(rk3588_soc.pwm3_base + mmap_seek));

	return reg_val;
}

/*
 * Wirte value to register helper
 */
void rk3588_write_reg(uint32_t addr, uint32_t val)
{
	uint32_t mmap_base;
	uint32_t mmap_seek;

	mmap_base = (addr & (0xfffff000));
	mmap_seek = (addr - mmap_base);

	if (mmap_base == RK3588_GPIO0_BASE)
		*((uint32_t*)(rk3588_soc.gpio0_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_GPIO1_BASE)
		*((uint32_t*)(rk3588_soc.gpio1_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_GPIO2_BASE)
		*((uint32_t*)(rk3588_soc.gpio2_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_GPIO3_BASE)
		*((uint32_t*)(rk3588_soc.gpio3_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_GPIO4_BASE)
		*((uint32_t*)(rk3588_soc.gpio4_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PMU1_IOC_BASE)
		*((uint32_t*)(rk3588_soc.pmu1_ioc_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PMU2_IOC_BASE)
		*((uint32_t*)(rk3588_soc.pmu2_ioc_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_BUS_IOC_BASE)
		*((uint32_t*)(rk3588_soc.bus_ioc_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_CRU_BASE)
		*((uint32_t*)(rk3588_soc.cru_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PMU1CRU_BASE)
		*((uint32_t*)(rk3588_soc.pmu1cru_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_VCCIO1_4_IOC_BASE)
		*((uint32_t*)(rk3588_soc.vccio1_4_ioc_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_VCCIO3_5_IOC_BASE)
		*((uint32_t*)(rk3588_soc.vccio3_5_ioc_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_VCCIO6_IOC_BASE)
		*((uint32_t*)(rk3588_soc.vccio6_ioc_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PWM0_BASE)
		*((uint32_t*)(rk3588_soc.pwm0_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PWM1_BASE)
		*((uint32_t*)(rk3588_soc.pwm1_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PWM2_BASE)
		*((uint32_t*)(rk3588_soc.pwm2_base + mmap_seek)) = val;
	else if (mmap_base == RK3588_PWM3_BASE)
		*((uint32_t*)(rk3588_soc.pwm3_base + mmap_seek)) = val;
}


static int rk3588_get_pwm_config(int pin, pwm_config_t *config)
{
	config->pwm_mux_addr = RK3588_BUS_IOC_BASE + ((pin >> 2) << 2);
	config->pwm_mux_value = 0x0B;
	config->pwm_mux_offset = (pin & 0x03) << 2;

	if (pin == 15 || pin == 16 || pin == 28) {
		config->pwm_mux_addr = RK3588_PMU2_IOC_BASE + (((pin >> 2) - 3) << 2);
		config->pwm_mux_value = 0x03;
	}

	switch (pin) {
		case 15:
		case 34:
		case 58:  //PWM0CH0
			config->pwm_base = RK3588_PWM0_BASE;
			config->ch_period_hpr = RK3588_CH0_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH0_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH0_CTRL(config->pwm_base);
			break;
		case 16:
		case 35:
		case 59:  //PWM0CH1
			config->pwm_base = RK3588_PWM0_BASE;
			config->ch_period_hpr = RK3588_CH1_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH1_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH1_CTRL(config->pwm_base);
			break;
		case 39:
		case 28:
		case 50:  //PWM0CH3
			config->pwm_base = RK3588_PWM0_BASE;
			config->ch_period_hpr = RK3588_CH3_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH3_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH3_CTRL(config->pwm_base);
			break;
		case 96:  //PWM2CH2
			config->pwm_base = RK3588_PWM2_BASE;
			config->ch_period_hpr = RK3588_CH2_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH2_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH2_CTRL(config->pwm_base);
			break;
		case 97:  //PWM2CH3
			config->pwm_base = RK3588_PWM2_BASE;
			config->ch_period_hpr = RK3588_CH3_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH3_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH3_CTRL(config->pwm_base);
			break;
		case 109:  //PWM3CH0
			config->pwm_base = RK3588_PWM3_BASE;
			config->ch_period_hpr = RK3588_CH0_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH0_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH0_CTRL(config->pwm_base);
			break;
		case 47:
		case 110:  //PWM3CH1
			config->pwm_base = RK3588_PWM3_BASE;
			config->ch_period_hpr = RK3588_CH1_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH1_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH1_CTRL(config->pwm_base);
			break;
		case 62:
		case 114:
		case 138:  //PWM3CH2
			config->pwm_base = RK3588_PWM3_BASE;
			config->ch_period_hpr = RK3588_CH2_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH2_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH2_CTRL(config->pwm_base);
			break;
		case 54:
		case 63:
		case 139:  //PWM3CH3
			config->pwm_base = RK3588_PWM3_BASE;
			config->ch_period_hpr = RK3588_CH3_PERIOD_HPR(config->pwm_base);
			config->ch_duty_lpr = RK3588_CH3_DUTY_LPR(config->pwm_base);
			config->ch_crtl = RK3588_CH3_CTRL(config->pwm_base);
			break;
		default:
			return -1;
	}

	return 0;
}

/*
 * Setup soc register
 */
int rk3588_setupReg(int fd)
{
	rk3588_soc.gpio0_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_GPIO0_BASE);
	if (rk3588_soc.gpio0_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_GPIO0_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.gpio1_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_GPIO1_BASE);
	if (rk3588_soc.gpio1_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_GPIO1_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.gpio2_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_GPIO2_BASE);
	if (rk3588_soc.gpio2_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_GPIO2_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.gpio3_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_GPIO3_BASE);
	if (rk3588_soc.gpio3_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_GPIO3_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.gpio4_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_GPIO4_BASE);
	if (rk3588_soc.gpio4_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_GPIO4_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pmu1_ioc_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PMU1_IOC_BASE);
	if (rk3588_soc.pmu1_ioc_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PMU1_IOC_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pmu2_ioc_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PMU2_IOC_BASE);
	if (rk3588_soc.pmu2_ioc_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PMU2_IOC_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.bus_ioc_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_BUS_IOC_BASE);
	if (rk3588_soc.bus_ioc_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_BUS_IOC_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.cru_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_CRU_BASE);
	if (rk3588_soc.cru_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_CRU_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pmu1cru_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PMU1CRU_BASE);
	if (rk3588_soc.pmu1cru_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PMU1CRU_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.vccio1_4_ioc_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_VCCIO1_4_IOC_BASE);
	if (rk3588_soc.vccio1_4_ioc_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_VCCIO1_4_IOC_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.vccio3_5_ioc_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_VCCIO3_5_IOC_BASE);
	if (rk3588_soc.vccio3_5_ioc_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_VCCIO3_5_IOC_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.vccio6_ioc_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_VCCIO6_IOC_BASE);
	if (rk3588_soc.vccio6_ioc_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_VCCIO6_IOC_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pwm0_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PWM0_BASE);
	if (rk3588_soc.pwm0_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PWM0_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pwm1_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PWM1_BASE);
	if (rk3588_soc.pwm1_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PWM1_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pwm2_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PWM2_BASE);
	if (rk3588_soc.pwm2_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PWM2_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	rk3588_soc.pwm3_base = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, RK3588_PWM3_BASE);
	if (rk3588_soc.pwm3_base == MAP_FAILED) {
		fprintf (stderr, "%s: mmap (RK3588_PWM3_BASE) failed: %s\n", __func__, strerror(errno));
		return -errno;
	}

	return 0;
}

/*
 * Set pin mask
 */
int rk3588_setPinMask(int model)
{
	if (model == PI_MODEL_HAIBOX_5A) {
		rk3588_soc.pin_mask = (pin_group_t *)haibox_5a_pin_mask;
	} else {
		rk3588_soc.pin_mask = NULL;
	}

	return 0;
}

/*
 * Returns the pin mode for a given port
 */
int rk3588_getPinMode(int pin)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t ddr_phyaddr;
	uint32_t pmu1_ioc_phyaddr;
	uint32_t bus_ioc_phyaddr;
	uint32_t reg_val;
	uint32_t iomux_sel;
	int pin_mode;

	if (!rk3588_soc.pin_mask || rk3588_soc.pin_mask[bank][index] == -1)
		return -1;

	bus_ioc_phyaddr = RK3588_BUS_IOC_BASE + ((pin >> 2) << 2);

	if (bank == 0) {
		if (index >= 12)
			pmu1_ioc_phyaddr = RK3588_PMU2_IOC_BASE + ((index - 12) >> 2 << 2);
		else
			pmu1_ioc_phyaddr = RK3588_PMU1_IOC_BASE + (index >> 2 << 2);
		ddr_phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
	}
	else if (bank == 1) {
		ddr_phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
	}
	else if (bank == 2) {
		ddr_phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
	}
	else if (bank == 3) {
		ddr_phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
	}
	else if (bank == 4) {
		ddr_phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
	} else {
		fprintf (stderr, "pin num(%d) is invalid\n", pin);
		return -1;
	}

	if (bank == 0) {
		reg_val = rk3588_read_reg(pmu1_ioc_phyaddr);
		iomux_sel = (reg_val >> ((index & 0x03) << 2)) & 0x0F;
		if (iomux_sel == 0x08)
			reg_val = rk3588_read_reg(bus_ioc_phyaddr);
	}
	else {
		reg_val = rk3588_read_reg(bus_ioc_phyaddr);
	}

	pin_mode = (reg_val >> ((index & 0x03) << 2)) & 0x0F;//获取控制模式的那四位的值
	if (pin_mode == 0) { //如果是gpio模式
		reg_val = rk3588_read_reg(ddr_phyaddr);	//获取gpio方向寄存器的值
		return (reg_val >> (index & 0x0F)) & 0x01;	//index对应的gpio的方向值，0为in，1为out
	}

	return pin_mode + 1; //如果不是gpio模式，返回的alt，从2开始，0和1是in和out
}

/*
 * Set pin mode
 */
int rk3588_setPinMode(int pin, int mode)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t ddr_phyaddr;
	uint32_t cru_phyaddr;
	uint32_t bus_ioc_phyaddr;
	uint32_t cru_val;
	uint32_t reg_val;
	pwm_config_t pwm_config;

	if (!rk3588_soc.pin_mask || rk3588_soc.pin_mask[bank][index] == -1)
		return -1;

	bus_ioc_phyaddr = RK3588_BUS_IOC_BASE + ((pin >> 2) << 2);

	if (bank == 0) {
		if (index >= 12)
			bus_ioc_phyaddr = RK3588_PMU2_IOC_BASE + ((index - 12) >> 2 << 2);
		else
			bus_ioc_phyaddr = RK3588_PMU1_IOC_BASE + (index >> 2 << 2);
		ddr_phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_PMU1CRU_GATE_CON5;
		cru_val = 0xFFFF9FFF;
	}
	else if (bank == 1) {
		ddr_phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON16;
		cru_val = 0xFFFF3FFF;
	}
	else if (bank == 2) {
		ddr_phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON17;
		cru_val = 0xFFFFFFFC;
	}
	else if (bank == 3) {
		ddr_phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON17;
		cru_val = 0xFFFFFFF3;
	}
	else if (bank == 4) {
		ddr_phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_SWPORT_DDR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON17;
		cru_val = 0xFFFFFFCF;
	}
	else {
		fprintf (stderr, "pin num(%d) is invalid\n", pin);
		return -1;
	}

	if (mode == INPUT) {
		reg_val = rk3588_read_reg(cru_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val &= cru_val;
		rk3588_write_reg(cru_phyaddr, reg_val);

		reg_val = rk3588_read_reg(bus_ioc_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val &= ~(0x0F << ((index & 0x03) << 2));
		rk3588_write_reg(bus_ioc_phyaddr, reg_val);

		reg_val = rk3588_read_reg(ddr_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val &= ~(1 << (index & 0x0F));
		rk3588_write_reg(ddr_phyaddr, reg_val);
	} 
	else if (mode == OUTPUT) {
		reg_val = rk3588_read_reg(cru_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val &= cru_val;
		rk3588_write_reg(cru_phyaddr, reg_val);

		reg_val = rk3588_read_reg(bus_ioc_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val &= ~(0x0F << ((index & 0x03) << 2));
		rk3588_write_reg(bus_ioc_phyaddr, reg_val);

		reg_val = rk3588_read_reg(ddr_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val |= (1 << (index & 0x0F));
		rk3588_write_reg(ddr_phyaddr, reg_val);
	}
	else if (mode == PWM_OUTPUT) {
		//set clk——busioc
		reg_val = rk3588_read_reg(RK3588_CRU_GATE_CON19);
		reg_val &= 0xFFFFFFFE;
		rk3588_write_reg(RK3588_CRU_GATE_CON19, reg_val);

		//Set clk——pwm123
		reg_val = rk3588_read_reg(RK3588_CRU_GATE_CON15);
		reg_val &= 0xFFFFF007;
		reg_val |= 0xFFFF0920;
		rk3588_write_reg(RK3588_CRU_GATE_CON15, reg_val);

		//Set clk——pmu1pwm
		reg_val = rk3588_read_reg(RK3588_PMU1CRU_GATE_CON1);
		reg_val &= 0xFFFF8FFF;
		reg_val |= 0xFFFF4000;
		rk3588_write_reg(RK3588_PMU1CRU_GATE_CON1, reg_val);

		if (rk3588_get_pwm_config(pin, &pwm_config) < 0) {
			fprintf (stderr, "pin(%d) does not support PWM mode\n", pin);
			return -1;
		}

		//Set mux
		reg_val = rk3588_read_reg(pwm_config.pwm_mux_addr);
		reg_val |= 0xFFFF0000;
		reg_val &= ~(0x0F << pwm_config.pwm_mux_offset);
		reg_val |= (pwm_config.pwm_mux_value << pwm_config.pwm_mux_offset);
		rk3588_write_reg(pwm_config.pwm_mux_addr, reg_val);

		//clear all reg
		rk3588_write_reg(pwm_config.ch_period_hpr, 0);
		rk3588_write_reg(pwm_config.ch_duty_lpr, 0);
		rk3588_write_reg(pwm_config.ch_crtl, 0);

		//Set period
		reg_val = rk3588_read_reg(pwm_config.ch_period_hpr);
		reg_val = 1000;
		rk3588_write_reg(pwm_config.ch_period_hpr, reg_val);

		//Set duty
		reg_val = rk3588_read_reg(pwm_config.ch_duty_lpr);
		reg_val = 500;
		rk3588_write_reg(pwm_config.ch_duty_lpr, reg_val);

		//Set ctrl
		/**
		* frequency of clock: 24 MHz / 120 = 200 kHz
		* frequency of PWM: 200 kHz / 1000 = 200 Hz
		* period of PWM: 1 / 200 Hz = 0.005 s
		* duty: 500/1000 = 50%
		*/
		reg_val = rk3588_read_reg(pwm_config.ch_crtl);
		reg_val = (0 << PWM_CTRL_RPT) \
				| (60 << PWM_CTRL_SCALE) \
				| (0 << PWM_CTRL_PRESCALE) \
				| (0 << PWM_CTRL_CLK_SRC_SEL) \
				| (1 << PWM_CTRL_CLK_SEL) \
				| (1 << PWM_CTRL_FORCE_CLK_EN) \
				| (1 << PWM_CTRL_CH_CNT_EN) \
				| (0 << PWM_CTRL_CONLOCK) \
				| (0 << PWM_CTRL_OUTPUT_MODE) \
				| (0 << PWM_CTRL_INACTIVE_POL) \
				| (1 << PWM_CTRL_DUTY_POL) \
				| (1 << PWM_CTRL_PWM_MODE) \
				| (1 << PWM_CTRL_PWM_EN);
		rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	}

	return 0;
}

/*
 *	Set the internal pull-up/down resistors of pin.
 */
int rk3588_setPullUpDn(int pin, int pud)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t ioc_phyaddr;
	uint32_t pin_offset;
	uint32_t reg_val;

	if (!rk3588_soc.pin_mask || rk3588_soc.pin_mask[bank][index] == -1)
		return -1;

	if(bank == 0 && index < 12)
		ioc_phyaddr = RK3588_PMU1_IOC_GPIO0A_P + ((index >> 3) << 2);
	else if(bank == 0 && index > 11 && index < 31)
		ioc_phyaddr = RK3588_PMU2_IOC_GPIO0B_P + (((index - 8) >> 3) << 2);
	else if(bank == 1)
		ioc_phyaddr = RK3588_VCCIO1_4_IOC_GPIO1A_P + ((index >> 3) << 2);
	else if(bank < 4 || (bank == 4 && index > 17))
		ioc_phyaddr = RK3588_VCCIO3_5_IOC_GPIO2A_P + (((pin - 64) >> 3) << 2);
	else if(bank == 4 && index < 18)
		ioc_phyaddr = RK3588_VCCIO6_IOC_GPIO4A_P + ((index >> 3) << 2);
	else {
		fprintf (stderr, "pin num(%d) is invalid\n", pin);
		return -1;
	}

	pin_offset = (index & 0x07) << 1;
	reg_val = rk3588_read_reg(ioc_phyaddr);

	/* clear bit */
	reg_val &= ~(3 << pin_offset);

	/* bit write enable*/
	reg_val |= 3 << (16 + pin_offset);

	/* set bit */
	/* */if (pud == PUD_UP)
		reg_val |= 3 << pin_offset;
	else if (pud == PUD_DOWN)
		reg_val |= 1 << pin_offset;

	rk3588_write_reg(ioc_phyaddr, reg_val);
	reg_val = rk3588_read_reg(ioc_phyaddr);

	return 0;
}

/*
 * Read digital pin
 */
int rk3588_digitalRead(int pin)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	uint32_t phyaddr = 0;

	if (!rk3588_soc.pin_mask || rk3588_soc.pin_mask[bank][index] == -1)
		return -1;

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
	else {
		fprintf (stderr, "pin num(%d) is invalid\n", pin);
		return -1;
	}

	return (rk3588_read_reg(phyaddr) >> index) & 0x01;
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

	if (!rk3588_soc.pin_mask || rk3588_soc.pin_mask[bank][index] == -1)
		return -1;

	if (bank == 0) {
		dr_phyaddr = RK3588_GPIO0_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_PMU1CRU_GATE_CON5;
		cru_val = 0xFFFF9FFF;
	}
	else if (bank == 1) {
		dr_phyaddr = RK3588_GPIO1_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON16;
		cru_val = 0xFFFF3FFF;
	}
	else if (bank == 2) {
		dr_phyaddr = RK3588_GPIO2_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON17;
		cru_val = 0xFFFFFFFC;
	}
	else if(bank == 3){
		dr_phyaddr = RK3588_GPIO3_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON17;
		cru_val = 0xFFFFFFF3;
	}
	else if (bank == 4) {
		dr_phyaddr = RK3588_GPIO4_BASE + RK3588_GPIO_SWPORT_DR_L_OFFSET + ((index >> 4) << 2);
		cru_phyaddr = RK3588_CRU_GATE_CON17;
		cru_val = 0xFFFFFFCF;
	}
	else {
		fprintf (stderr, "pin num(%d) is invalid\n", pin);
		return -1;
	}

	reg_val = rk3588_read_reg(cru_phyaddr);
	reg_val &= cru_val;
	rk3588_write_reg(cru_phyaddr, reg_val);

	if (!value) {
		reg_val = rk3588_read_reg(dr_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val &= ~(1 << (index & 0x0F));
		rk3588_write_reg(dr_phyaddr, reg_val);
		reg_val = rk3588_read_reg(dr_phyaddr);
	} else {
		reg_val = rk3588_read_reg(dr_phyaddr);
		reg_val |= 0xFFFF0000;
		reg_val |= (1 << (index & 0x0F));
		rk3588_write_reg(dr_phyaddr, reg_val);
		reg_val = rk3588_read_reg(dr_phyaddr);
	}

	return 0;
}

/*
 * Set output PWM value
 */
int rk3588_pwmWrite(int pin, unsigned int value)
{
	uint32_t reg_val;
	pwm_config_t pwm_config;

	if (rk3588_get_pwm_config(pin, &pwm_config) < 0) {
		fprintf (stderr, "pin(%d) does not support PWM mode\n", pin);
		return -1;
	}

	reg_val = rk3588_read_reg(pwm_config.ch_period_hpr);
	if (value > reg_val) {
		fprintf (stderr, "gpio: CCR should be less than or equal to ARR (%d)\n", reg_val);
		return -1;
	}

	//enable conlock
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val |= (1 << PWM_CTRL_CONLOCK);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	//modifine act
	rk3588_write_reg(pwm_config.ch_duty_lpr, value);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_duty_lpr);

	//disable conlock
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val &= ~(1 << PWM_CTRL_CONLOCK);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	return 0;
}

/*
 * Set output PWM clock
 */
int rk3588_pwmClock(int pin, unsigned int divisor)
{
	uint32_t reg_val;
	pwm_config_t pwm_config;

	if (rk3588_get_pwm_config(pin, &pwm_config) < 0) {
		fprintf (stderr, "pin(%d) does not support PWM mode\n", pin);
		return -1;
	}

	if (divisor == 0 || divisor > PWM_CTRL_SCALE_MAX) {
		fprintf (stderr, "pwm clk(%d) divisor must be 1 ~ %d\n", divisor, PWM_CTRL_SCALE_MAX);
		return -1;
	}

	if (divisor >= PWM_CTRL_SCALE_MAX) {
		divisor = 0;
	} else if (divisor > 1) {
		divisor >>= 1;
	}

	//enable conlock
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val |= (1 << PWM_CTRL_CONLOCK);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	//modifine scale
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val &= 0xFF00FFFF;
	reg_val |= (divisor << PWM_CTRL_SCALE);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	//disable conlock
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val &= ~(1 << PWM_CTRL_CONLOCK);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	return 0;
}

/*
 * Set output PWM range
 */
int rk3588_pwmRange(int pin, unsigned int range)
{
	uint32_t reg_val;
	pwm_config_t pwm_config;

	if (rk3588_get_pwm_config(pin, &pwm_config) < 0) {
		fprintf (stderr, "pin(%d) does not support PWM mode\n", pin);
		return -1;
	}

	if (range == 0) {
		fprintf (stderr, "pwm range must be > 0\n");
		return -1;
	}

	range = (range + 1) & 0xFFFFFFFE;

	//enable conlock
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val |= (1 << PWM_CTRL_CONLOCK);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	//modifine period
	rk3588_write_reg(pwm_config.ch_period_hpr, range);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_period_hpr);

	//disable conlock
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);
	reg_val &= ~(1 << PWM_CTRL_CONLOCK);
	rk3588_write_reg(pwm_config.ch_crtl, reg_val);
	RK_DELAY(1);
	reg_val = rk3588_read_reg(pwm_config.ch_crtl);

	return 0;
}