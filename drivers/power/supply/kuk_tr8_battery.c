/*
 * linux/drivers/power/kuk_tr8_battery.c
 *
 * Battery measurement code for trizeps8/(mini)
 *
 * based on tosa_battery.c
 *
 * Copyright (C) 2017 Scheidt & Bachmann GmbH / Keith & Koep GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/sort.h>
#include <linux/of_gpio.h>

#define  FAKE_ADC_SAMPLE        0	// TODO

#define  FAKE_ADC_READING       0xfff
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
#define CONFIG_IPAN5_SB		1
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
#define ADC_SAMPLE_COUNT	6
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
#define	DELAY			1
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
#define TOUT_BATCHANGE		60
#define TOUT_BATCHANGE_WARN	50
#define BAT_CHANGE		1

struct tr8_batt_pdata {
	int	batt_aux;
	int	temp_aux;
	int	charge_gpio;
	int	charging_low;
	int	min_voltage;
	int	max_voltage;
	int	batt_div;
	int	batt_mult;
	int	temp_div;
	int	temp_mult;
	int	batt_tech;
	char	*batt_name;
	// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + @+23.06.2016 SuB HL
	int	backup_batt_aux;
	int	backup_batt_div;
	int	backup_batt_mult;
	int	backup_charge_gpio;
	int external_power_gpio;
	char	*backup_batt_name;
};

struct tr8_pdata {
  struct tr8_batt_pdata	*batt_pdata;	/* battery data */
  char                  *tr8_batt_drv;  /* copied after KUKTR8BAT_get_platform_battery_data by trizeps7_probe (soc/imx/trizeps-wm9715.c) */
};

static struct tr8_pdata *g_tr8data = NULL;

static DEFINE_MUTEX(bat_lock);
static struct delayed_work bat_delayed_work;
static struct work_struct bat_work;
static struct mutex work_lock;
static int bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
static enum power_supply_property *prop;


// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
static int offset_discharger 	= 0;
static int offset_charger 	= 0;
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL

static int voltage_uV     =11000000;
static int voltage_temp_uV=1024;	//     20.11.2014 S&B HH/HL
static int charger_online = 0;
static int percent;
static int old_percent;

static int batChangeGpio;
static int batChangeCount = TOUT_BATCHANGE_WARN / DELAY;

static int yMutexIsInit = false;	//		@+08.05.2018 S&B WR/HL

typedef struct {
	u32 voltage;
	u32 percent;
} battery_capacity , *pbattery_capacity;
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 24.11.2014 S&B HH/HL
static battery_capacity chargingTable[] = {
	{12000000,	100},
	{11879378,	99},
	{11757097,	98},
	{11637631,	97},
	{11510854,	96},
	{11402555,	95},
	{11270279,	94},
	{11152185,	93},
	{11014980,	92},
	{10911385,	91},
	{10796894,	90},
	{10205177,	85},
	{9695401,	81},
	{9185157,	77},
	{8729618,	73},
	{8385669,	70},
	{7800747,	65},
	{7121164,	60},
	{6608052,	55},
	{6040976,	50},
	{5414260,	45},
	{4721632,	40},
	{3956159,	32},
	{3543743,	30},
	{3110181,	26},
	{2654391,	22},
	{2175231,	18},
	{1671504,	13},
	{1141951,	9},
	{ 585247,	5},
	{ 470527,	4},
	{      0,	0},
};
static battery_capacity dischargingTable[] = {
	{12000000,     100},
	{11880598,	99},
	{11762384,	98},
	{11645346,	97},
	{11529473,	96},
	{11414753,	95},
	{10885049,	90},
	{10328496,	86},
	{9824769,	82},
	{9345609,	78},
	{8889819,	74},
	{8456257,	69},
	{8043841,	67},
	{7278368,	61},
	{6585740,	55},
	{5959024,	50},
	{5391948,	45},
	{4640892,	39},
	{4199253,	35},
	{3438058,	29},
	{2959164,	25},
	{3270382,	27},
	{2304599,	19},
	{1886846,	16},
	{1707289,	14},
	{1544819,	13},
	{1397810,	12},
	{1329638,	11},
	{1203106,	10},
	{1088615,	9},
	{985020,	8},
	{847815,	7},
	{729721,	6},
	{597445,	5},
	{489146,	4},
	{362369,	3},
	{242903,	2},
	{120622,	1},
	{0,	0},
};
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 24.11.2014 S&B HH/HL

//------------------------------------------------------------------------
#if 0
static int AndroidBatChangeNotify(void)
{
	if(gpio_get_value(batChangeGpio) == BAT_CHANGE) {
	}	
	else {
	}
	return 0;
}
#endif
//------------------------------------------------------------------------
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 08.05.2018 S&B WR/HL
// Export small functions 4 other drivers
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 08.05.2018 S&B WR/HL
int iPowerGetCapacity(void)
{
	int tPercent;

	if(yMutexIsInit){
		mutex_lock(&work_lock);
		tPercent = percent;
		mutex_unlock(&work_lock);
		return tPercent;
	}
	else
		return 0;
}
EXPORT_SYMBOL_GPL(iPowerGetCapacity);

int iPowerGetVoltage_uV(void)
{
	int tVoltage_uV;

	if(yMutexIsInit){
		mutex_lock(&work_lock);
		tVoltage_uV = voltage_uV;
		mutex_unlock(&work_lock);
		return tVoltage_uV;
	}
	else
		return 0;
}
EXPORT_SYMBOL_GPL(iPowerGetVoltage_uV);

// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 08.05.2018 S&B WR/HL
struct i2c_client *tr8_adc_get_handle(void);
int tr8_read_adc(int adcsel);
struct i2c_client *tr8_adc_get_i2c_client(void);


static unsigned long tr8_read_bat(struct power_supply *bat_ps)
{
	struct tr8_pdata      *tr8data = g_tr8data;
	struct tr8_batt_pdata *pdata   = tr8data->batt_pdata;
#if FAKE_ADC_SAMPLE
	return FAKE_ADC_READING * pdata->batt_mult / pdata->batt_div;
#else
	return tr8_read_adc(pdata->batt_aux)*pdata->batt_mult/pdata->batt_div;	
#endif
}


//------------------------------------------------------------------------
static unsigned long tr8_read_temp(struct power_supply *bat_ps)
{
	struct tr8_pdata      *tr8data = g_tr8data;
	struct tr8_batt_pdata *pdata   = tr8data->batt_pdata;
#if FAKE_ADC_SAMPLE
	return FAKE_ADC_READING * pdata->temp_mult / pdata->temp_div;
#else
	if (pdata->temp_aux >= 0)
	  return tr8_read_adc(pdata->temp_aux)*pdata->temp_mult/pdata->temp_div;
	else
	  return(-1);	
#endif
}


//------------------------------------------------------------------------
static int tr8_bat_get_property(struct power_supply *bat_ps,
			    enum power_supply_property psp,
			    union power_supply_propval *val)
{
	struct tr8_pdata *tr8data = g_tr8data;
	struct tr8_batt_pdata *pdata = tr8data->batt_pdata;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (bat_status == POWER_SUPPLY_STATUS_FULL)
		  val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (percent <= 15)
		  val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else
		  val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		printk (KERN_ERR "##### KUKTR8BATXX - CAPACITY_LEVEL:%d \n", val->intval);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
	        val->intval = (percent < 0) ? 0:((percent>100)?100:percent);
	        printk (KERN_ERR "##### KUKTR8BATXX - CAPACITY:%d \n", val->intval);
		break;	

	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		printk (KERN_ERR "##### KUKTR8BATXX - STATUS:%d \n", val->intval);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->batt_tech;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (pdata->batt_aux >= 0)
			val->intval = voltage_uV;
		else
			return -EINVAL;
 printk (KERN_ERR "##### KUKTR8BATXX - VOLTAGE_NOW:%d \n", val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (pdata->temp_aux >= 0)
			val->intval = voltage_temp_uV;			// 20.11.2014 S&B HH/HL
		else
		  return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (pdata->max_voltage >= 0)
			val->intval = pdata->max_voltage;
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		if (pdata->min_voltage >= 0)
			val->intval = pdata->min_voltage;
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


//------------------------------------------------------------------------
#if 0
static bool isCharging(struct power_supply *bat_ps)
{
	unsigned long val; 

	val = tr8_read_temp(bat_ps);
	printk (KERN_ERR "##### KUKTR8BATXX ------------------- Charging:%lu \n", val);
	return (val < 100);
}
#endif

//------------------------------------------------------------------------
static u32 get_capability_percent(struct tr8_batt_pdata *pdata)
{
	u8 i;
	pbattery_capacity pTable;
	u32 tableSize;	
	
	if (bat_status  == POWER_SUPPLY_STATUS_DISCHARGING) {
		pTable = dischargingTable;
		tableSize = sizeof(dischargingTable)/sizeof(dischargingTable[0]);
	} else {
		pTable = chargingTable;
		tableSize = sizeof(chargingTable)/sizeof(chargingTable[0]);
	}
	for (i = 0; i < tableSize; i++) {
		if (voltage_uV >= pTable[i].voltage)
			return	pTable[i].percent;
	}
	return 0;
}


//------------------------------------------------------------------------
static int cmp_func(const void *_a, const void *_b)
{
	const int *a = _a, *b = _b;

	if (*a > *b)
		return 1;
	if (*a < *b)
		return -1;
	return 0;
}


//------------------------------------------------------------------------
static u32 get_voltage(struct power_supply *bat_ps)
{
	int volt[ADC_SAMPLE_COUNT];
	u32 voltage_data;
	int i;

	for (i = 0; i < ADC_SAMPLE_COUNT; i++) 
	{
		if (charger_online == 0) 
		{
			/* ADC offset when battery is discharger*/
			volt[i] = tr8_read_bat(bat_ps)-offset_discharger;
		} 
		else 
		{
			volt[i] = tr8_read_bat(bat_ps)-offset_charger;
		}
	}
	sort(volt, i, 4, cmp_func, NULL);
	voltage_data = (volt[3] + volt[ADC_SAMPLE_COUNT - 2]) / 2;
	return voltage_data;
}

// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
static u32 get_voltage_temp(struct power_supply *bat_ps)
{
	int volt[ADC_SAMPLE_COUNT];
	u32 voltage_data;
	int i;

	for (i = 0; i < ADC_SAMPLE_COUNT; i++) 
	{
		volt[i] = tr8_read_temp(bat_ps);
	}
	sort(volt, i, 4, cmp_func, NULL);
	voltage_data = (volt[3] + volt[ADC_SAMPLE_COUNT - 2]) / 2;
	return voltage_data;
}
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL


//------------------------------------------------------------------------
static void tr8_bat_external_power_changed(struct power_supply *bat_ps)
{
  printk (KERN_ERR "######## KUKTR8BATXX - EXTERN POWER CHANGED\n");
  schedule_work(&bat_work);
}


//------------------------------------------------------------------------
static void tr8_bat_update(struct power_supply *bat_ps)
{
	struct tr8_pdata *tr8data = g_tr8data;
	struct tr8_batt_pdata *pdata = tr8data->batt_pdata;
	int old_status = bat_status;
	int temp = 0;
	static int temp_last;
	bool changed_flag = false;
	int tpercent, new_status;	

	mutex_lock(&work_lock);
	
	voltage_temp_uV = get_voltage_temp(bat_ps);					// 20.11.2014 S&B HH/HL

	temp = get_voltage(bat_ps);
	if (temp_last == 0) {
		voltage_uV = temp;
		temp_last = temp;
	}
	if (charger_online == 0 && temp_last != 0) 
	{
		temp_last = temp;
		voltage_uV = temp;
	}
	if (charger_online == 1) {
		voltage_uV = temp;
		temp_last = temp;
	}

	changed_flag      = false;
	tpercent= percent = get_capability_percent(pdata);

#ifdef POWER_EVER_GOOD 
	batChange =0;
#endif

	if (percent != old_percent) {
	  if( abs(old_percent-percent) > 2 )
	  {
		changed_flag = true;	      
		old_percent = percent;
	  }	  
	}
	new_status =  POWER_SUPPLY_STATUS_UNKNOWN;
	if( pdata->charge_gpio >= 0 )
	{
	  new_status = POWER_SUPPLY_STATUS_DISCHARGING;
	  if(  pdata->charging_low && (gpio_get_value(pdata->charge_gpio)==0))
	    new_status = POWER_SUPPLY_STATUS_CHARGING;

	  if( (pdata->charging_low==0) && gpio_get_value(pdata->charge_gpio))
	      new_status = POWER_SUPPLY_STATUS_CHARGING;
	}
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 24.11.2014 S&B HH/HL
	else
	{
		if(voltage_temp_uV > 1000000)	                 // Power supply voltage >0,1V
		  new_status = POWER_SUPPLY_STATUS_CHARGING;
		else	
		  new_status = POWER_SUPPLY_STATUS_DISCHARGING;	  
 	}
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 24.11.2014 S&B HH/HL
	bat_status=new_status;	
	// printk (KERN_ERR "##### KUKTR8BATXX - voltage:%d percent: %d count:%d status %d\n", voltage_uV, percent, batChangeCount, new_status);
	// ##### 130801 EG
	// The next line is only for testing, because of problems with AC97
	if(voltage_uV > 100000) // 100mV !?
	  if ((old_status != bat_status) || changed_flag)
	  {
	    //pr_debug("%s: %i -> %i\n", bat_ps->desc->name, old_status, bat_status);
#if 0
	    if( pdata->charge_gpio >= 0 )					// 20.11.2014 S&B HH/HL
	      printk (KERN_ERR "##### KUKTR8BATXX - POWER SUPPLY CHANGED %d percent, GPIO=%d\n", 
		      tpercent, gpio_get_value(pdata->charge_gpio));
#endif
	    power_supply_changed(bat_ps);
	  }
	mutex_unlock(&work_lock);
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
static struct power_supply *bat_psy;
static struct power_supply_desc bat_psy_desc = {
	.type				= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= tr8_bat_get_property,
	.external_power_changed = tr8_bat_external_power_changed,
	.use_for_apm		= 1,
};


//------------------------------------------------------------------------
static void tr8_bat_work(struct work_struct *work)
{
    if( tr8_adc_get_i2c_client() != NULL )
    {
      tr8_bat_update(bat_psy);
    }
}

//------------------------------------------------------------------------
static void tr8_bat_delayed_work(struct work_struct *work)
{
	schedule_work(&bat_work);	
	schedule_delayed_work(&bat_delayed_work, DELAY * HZ);
}


//------------------------------------------------------------------------
static irqreturn_t tr8_chrg_irq(int irq, void *data)
{
  printk (KERN_ERR "########## KUKTR8BATXX - IRQ - GpioBatChange\n");
	batChangeCount = 0;
	schedule_work(&bat_work);
//	AndroidBatChangeNotify();

	return IRQ_HANDLED;
}


//------------------------------------------------------------------------
#ifdef CONFIG_PM
static int tr8_bat_suspend(struct device *dev)
{
	cancel_delayed_work(&bat_delayed_work);
	flush_work(&bat_work);
	return 0;
}


//------------------------------------------------------------------------
static int tr8_bat_resume(struct device *dev)
{
	schedule_delayed_work(&bat_delayed_work, DELAY * HZ);
//	AndroidBatChangeNotify();

	return 0;
}


//------------------------------------------------------------------------
//------------------------------------------------------------------------
static const struct dev_pm_ops tr8_bat_pm_ops = {
	.suspend	= tr8_bat_suspend,
	.resume		= tr8_bat_resume,
};
#endif

//------------------------------------------------------------------------
#if defined(CONFIG_OF)
static const struct of_device_id tr8_bat_dt_ids[] = {
	{ .compatible = "sub,tr8_cap","kuk,tr8_bat" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tr8_bat_dt_ids);

#define POWER_SUPPLY_TECHNOLOGY_CAP POWER_SUPPLY_TECHNOLOGY_LiMn+1// Our capacitors USV is not defined, so we defined it here.

static struct tr8_pdata *tr8_bat_of_populate_pdata(struct device *dev)
{
	struct device_node    *of_node 	= dev->of_node;
	struct tr8_pdata      *tr8data 	= dev->platform_data;
	struct tr8_batt_pdata *pxBattData= NULL;

	if (!of_node || tr8data)
		return tr8data;

	tr8data = devm_kzalloc(dev, sizeof(struct tr8_pdata),
				GFP_KERNEL);
	if (!tr8data)
		return tr8data;

	pxBattData= devm_kzalloc(dev, sizeof(struct tr8_batt_pdata),
				GFP_KERNEL);
	if (!pxBattData)
		return NULL;

	tr8data->batt_pdata = pxBattData;

	/* Parse values of Device Tree */
	if (of_property_read_u32(of_node, "batt_aux", &pxBattData->batt_aux)){
		dev_err(dev, "Not setting batt_aux in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: batt_aux = 0x%x \n", pxBattData->batt_aux);

	if (of_property_read_u32(of_node, "temp_aux", &pxBattData->temp_aux)){
		dev_err(dev, "Not setting temp_aux in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: temp_aux = 0x%x \n", pxBattData->temp_aux);

	pxBattData->charge_gpio = of_get_named_gpio(of_node, "charge_gpio", 0);
	if (!gpio_is_valid(pxBattData->charge_gpio)) {
		dev_err(dev, "pin pxBattData->charge_gpio: invalid gpio %d\n", pxBattData->charge_gpio);
		pxBattData->charge_gpio = 0;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: charge_gpio = %d \n", pxBattData->charge_gpio);

	if (of_property_read_u32(of_node, "charging_low", &pxBattData->charging_low)){
		dev_err(dev, "Not setting charging_low in dts!\n");
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: charging_low = %d \n", pxBattData->charging_low);

	if (of_property_read_u32(of_node, "batt_mult", &pxBattData->batt_mult)){
		dev_err(dev, "Not setting batt_mult in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: batt_mult = %d \n", pxBattData->batt_mult);

	if (of_property_read_u32(of_node, "batt_div", &pxBattData->batt_div)){
		dev_err(dev, "Not setting batt_div in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: batt_div = %d \n", pxBattData->batt_div);

	if (of_property_read_u32(of_node, "temp_mult", &pxBattData->temp_mult)){
		dev_err(dev, "Not setting temp_mult in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: temp_mult = %d \n", pxBattData->temp_mult);

	if (of_property_read_u32(of_node, "temp_div", &pxBattData->temp_div)){
		dev_err(dev, "Not setting temp_div in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: temp_div = %d \n", pxBattData->temp_div);

	if (of_property_read_u32(of_node, "min_voltage", &pxBattData->min_voltage)){
		dev_err(dev, "Not setting min_voltage in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: min_voltage = %d \n", pxBattData->min_voltage);

	if (of_property_read_u32(of_node, "max_voltage", &pxBattData->max_voltage)){
		dev_err(dev, "Not setting max_voltage in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: max_voltage = %d \n", pxBattData->max_voltage);

	if (of_property_read_u32(of_node, "batt_tech", &pxBattData->batt_tech)){
		dev_err(dev, "Not setting batt_tech in dts!\n");
		return NULL;
	}
	else
		printk (KERN_ERR  "tr8_bat_of_populate_pdata: batt_tech = %d \n", pxBattData->batt_tech);

	if (of_property_read_string(of_node, "batt_name", (const char **)&pxBattData->batt_name)){
		dev_err(dev, "Not setting batt_name in dts!\n");
		pxBattData->batt_name		= "CapUSV";		 			// capacitors as USV [default]
	}
	printk (KERN_ERR  "tr8_bat_of_populate_pdata: batt_name = %s \n", pxBattData->batt_name);

	return tr8data;
}
#endif

static int tr8_bat_probe(struct platform_device *dev)
{
	int ret = 0;
	int props = 2;	/* POWER_SUPPLY_PROP_PRESENT u. POWER_SUPPLY_PROP_STATUS*/
	int i = 0;
//	struct tr8_pdata *tr8data = dev->dev.platform_data;
	struct tr8_pdata *tr8data = g_tr8data;
	struct tr8_batt_pdata *pdata;
	struct power_supply_config psy_cfg = {};


	if (!tr8data) {
		tr8data = tr8_bat_of_populate_pdata(&dev->dev);
		if(!tr8data){
			dev_err(&dev->dev, "No platform data supplied\n");
			return -EINVAL;
		}
		if(dev->dev.of_node)	// of settings given?
			g_tr8data = tr8data;
		printk (KERN_ERR  "tr8_bat_probe: 1st END after tr8_bat_of_populate_pdata()!!!!!!!!\n");
	}

	pdata = tr8data->batt_pdata;

//	platform_set_drvdata(dev, tr8data);  // Set it back to device struct

	if (dev->id != -1)
		return -EINVAL;


	if (!pdata) {
		dev_err(&dev->dev, "tr8_bat_probe: No platform_data supplied\n");
		return -EINVAL;
	}

	if ((pdata->charge_gpio) > 0 && gpio_is_valid(pdata->charge_gpio)) {
		ret = gpio_request(pdata->charge_gpio, "BATT CHRG");
		if (ret)
			goto err;
		ret = gpio_direction_input(pdata->charge_gpio);
		if (ret)
			goto err2;
		ret = request_irq(gpio_to_irq(pdata->charge_gpio),
				tr8_chrg_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"Battery change", dev);
		if (ret)
			goto err2;
		batChangeGpio = pdata->charge_gpio;
	}

	if (pdata->batt_tech >= 0)
		props++;	/* POWER_SUPPLY_PROP_TECHNOLOGY */
	if (pdata->batt_aux >= 0)
		props+=4;	/* POWER_SUPPLY_PROP_VOLTAGE_NOW */
	if (pdata->max_voltage >= 0)
		props++;	/* POWER_SUPPLY_PROP_VOLTAGE_MAX */
	if (pdata->min_voltage >= 0)
		props++;	/* POWER_SUPPLY_PROP_VOLTAGE_MIN */
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
	if (pdata->temp_aux >= 0)
		props++;	/* POWER_SUPPLY_PROP_TEMP */
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL

	prop = kzalloc(props * sizeof(*prop), GFP_KERNEL);
	if (!prop)
		goto err3;

	prop[i++] = POWER_SUPPLY_PROP_PRESENT;
	prop[i++] = POWER_SUPPLY_PROP_STATUS;
	if (pdata->batt_tech >= 0)
		prop[i++] = POWER_SUPPLY_PROP_TECHNOLOGY;
	if (pdata->batt_aux >= 0) {
		prop[i++] = POWER_SUPPLY_PROP_VOLTAGE_NOW;
		prop[i++] = POWER_SUPPLY_PROP_CAPACITY;
		prop[i++] = POWER_SUPPLY_PROP_CAPACITY_LEVEL;
		prop[i++] = POWER_SUPPLY_PROP_HEALTH;
	}
	if (pdata->max_voltage >= 0)
		prop[i++] = POWER_SUPPLY_PROP_VOLTAGE_MAX;
	if (pdata->min_voltage >= 0)
		prop[i++] = POWER_SUPPLY_PROP_VOLTAGE_MIN;
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL
	if (pdata->temp_aux >= 0)
		prop[i++] = POWER_SUPPLY_PROP_TEMP;
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + 20.11.2014 S&B HH/HL

	if (!pdata->batt_name) {
		dev_info(&dev->dev, "Please consider setting proper battery "
				"name in platform definition file, falling "
				"back to name \"TR8BAT-KUK\"\n");
		bat_psy_desc.name = "TR8BAT-batt-kuk";
	} else
		bat_psy_desc.name = pdata->batt_name;

	bat_psy_desc.properties = prop;
	bat_psy_desc.num_properties = props;

	psy_cfg.drv_data = tr8data;

	mutex_init(&work_lock);
	yMutexIsInit = true;																		//	@+08.05.2018 S&B WR/HL
	INIT_WORK(&bat_work, tr8_bat_work);
	INIT_DELAYED_WORK(&bat_delayed_work, tr8_bat_delayed_work);

//	bat_psy = power_supply_register(&dev->dev, &bat_psy_desc, &psy_cfg);
	bat_psy = power_supply_register(&dev->dev, &bat_psy_desc, NULL);

	if (!IS_ERR(bat_psy))
	{	    
	  schedule_delayed_work(&bat_delayed_work, DELAY * HZ);
	  // schedule_delayed_work(&bat_delayed_work, 60 * HZ);
	}else{
		ret = PTR_ERR(bat_psy);
		printk (KERN_ERR  "tr8_bat_probe: Error %d by power_supply_register() call!\n", ret);
		goto err4;
	}
//	AndroidBatChangeNotify();

	printk (KERN_ERR  "tr8_bat_probe: 2nd END!\n");
	return 0;
err4:
	printk (KERN_ERR  "tr8_bat_probe: err4 exit!\n");
	kfree(prop);
err3:
	printk (KERN_ERR  "tr8_bat_probe: err3 exit!\n");
	if (gpio_is_valid(pdata->charge_gpio))
		free_irq(gpio_to_irq(pdata->charge_gpio), dev);
err2:
	printk (KERN_ERR  "tr8_bat_probe: err2 exit!\n");
	if (gpio_is_valid(pdata->charge_gpio))
		gpio_free(pdata->charge_gpio);
err:
	printk (KERN_ERR  "tr8_bat_probe: END with error!\n");
	return ret;
}

static int tr8_bat_remove(struct platform_device *dev)
{
	struct tr8_pdata *tr8data = dev->dev.platform_data;
	struct tr8_batt_pdata *pdata = tr8data->batt_pdata;

	if (pdata && gpio_is_valid(pdata->charge_gpio)) {
		free_irq(gpio_to_irq(pdata->charge_gpio), dev);
		gpio_free(pdata->charge_gpio);
	}
	cancel_delayed_work_sync(&bat_delayed_work);
	cancel_work_sync(&bat_work);

	power_supply_unregister(bat_psy);
	kfree(prop);
	return 0;
}


//------------------------------------------------------------------------
//------------------------------------------------------------------------
static struct platform_driver tr8_bat_driver = {
	.driver		= {
		.name			= "tr8-battery",
		.owner			= THIS_MODULE,
		.of_match_table = tr8_bat_dt_ids,
#ifdef CONFIG_PM
		.pm		= &tr8_bat_pm_ops,
#endif
	},
	.probe		= tr8_bat_probe,
	.remove		= tr8_bat_remove,
};

module_platform_driver(tr8_bat_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Scheidt & Bachmann GmbH, Keith&Koep GmbH");
MODULE_DESCRIPTION("TR8BAT-KUK battery driver for Linux");
