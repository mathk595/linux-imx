/*	--*- c -*--
 * Copyright (C) 2014 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 or (at your option) version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/pagemap.h>

// #include <asm/system_info.h>
#include "../../arch/arm/mach-imx/mx6.h"
#include "../../arch/arm/mach-imx/hardware.h"
#include "board-kk_trizeps7.h"

unsigned int __mxc_cpu_type;
unsigned int system_rev;
unsigned int system_serial_low;
unsigned int system_serial_high;
unsigned int __mxc_cpu_type;
unsigned int system_rev;
unsigned int system_serial_low;
unsigned int system_serial_high;

#define KK_RSRVD_IN			 (1 << 0)
#define KK_RSRVD_OUT			 (0 << 0)
#define KK_RSRVD_CHANGEABLE              (1 << 6)
#define KK_RSRVD_OUT_LO			((0 << 1) | KK_RSRVD_OUT)
#define KK_RSRVD_OUT_HI			((1 << 1) | KK_RSRVD_OUT)
#define KK_RSRVD_REQUEST(_idx)		((1 << 8) | ((_idx) << 16))
#define KK_RSRVD_EXPORT			 (1 << 5)					// @+15.05.2017 S&B HL


struct gpio_data {
	char const	*label;
	unsigned int	gpio;
	unsigned int	flags;
	bool		do_release;
};

struct gpio_group {
	size_t			cnt;
	struct gpio_data	gpios[];
};


int       read_alternative_data=0;

static int trizeps7_reserved_read_gpio(struct device_node *np,
				       char const *name,
				       unsigned int idx,
				       struct gpio_group *grp)
{
	enum of_gpio_flags	flags;
	int			rc;
	struct gpio_data	*gpio;
	char const	        *gp_name;

	if( grp == NULL )
	  return -1;

	rc = of_get_named_gpio_flags(np, name, idx, &flags);
	if (rc < 0) {
                if (rc == -EPROBE_DEFER)
                        printk(KERN_ERR "%s: %s: GPIOs not yet available, retry later\n",
                                        np->name, __func__);
                else
                        printk(KERN_ERR "%s: %s: Can't get '%s' DT property\n",
                                        np->name, __func__, name);
		printk(KERN_ERR "%s: no gpio #%s #%u (->%d)\n", __func__, name, idx, rc);
		goto out;
        }

	gpio = &grp->gpios[grp->cnt];
	gpio->gpio  = rc;

	//printk(KERN_ERR "%s:Gpio %03d DTS-Flag 0x%02x\n",  __func__, gpio->gpio, flags);

	gpio->flags = ( ((flags&0xff) & KK_RSRVD_IN)  ?    GPIOF_IN :
		       ( (flags&0xff) & KK_RSRVD_OUT_HI) == KK_RSRVD_OUT_HI ?
		         GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW);
	
	if( flags & KK_RSRVD_CHANGEABLE)
	  gpio->flags |= GPIOF_EXPORT_CHANGEABLE;	
	// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + @+15.05.2017 S&B HL
	if(flags & KK_RSRVD_EXPORT)							// Export set?
		gpio->flags |= GPIOF_EXPORT;						// Yes!
	// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + + @+15.05.2017 S&B HL
	if (flags & KK_RSRVD_REQUEST(0))
	{
	        gpio->label = gp_name = "reserved?";	  
		gpio->do_release = false;
	} else {
		gpio->label = gp_name = "reserved";
		gpio->do_release = true;
	}
	{	  
	  rc = of_property_read_string_index(np, "kk,gpio-names", grp->cnt, &gp_name);	  
	  if (rc < 0)
	  {
	    printk(KERN_ERR "%s:failed to determine name for reserved gpio %u: %d\n",__func__, gpio->gpio, rc);
	    if( flags & KK_RSRVD_REQUEST(0) ) // RESERVED Requests must have Labels
	      goto out;
	    else
	      goto noname;	    
	  }
	  gpio->label = kstrdup(gp_name, GFP_KERNEL);
	  if (!gpio->label) {
	    printk(KERN_ERR "%s: failed to copy gpio name '%s'\n",
		   __func__, gp_name);
	    rc = -ENOMEM;
	    goto out;
	  }
	}
 noname:		
	grp->cnt += 1;
	rc = 0;
out:
	return rc;
}

static int trizeps7_reserved_read_gpio_group(struct device_node *np,
					  char const *name,
					  struct gpio_group **gpio_group)
{
	size_t			idx;
	int			rc;
	int			cnt;
	uint32_t const		*tmp;
	struct gpio_group	*grp = NULL;

	tmp = of_get_property(np, name, &cnt);
	if (!tmp) {
		printk(KERN_ERR "%s: no '%s' property found\n", __func__, name);
		return -ENOENT;
	}

	if (cnt % sizeof *tmp != 0) {
		printk(KERN_ERR "%s: "
		       "misaligned number (%d) of elements in gpio group\n",
		       __func__, cnt);
		return -EINVAL;
	}

	cnt /= sizeof *tmp;

	if (cnt % 3 != 0) {
		printk(KERN_ERR "%s: "
		       "odd number (%d) of elements in gpio group\n",
		       __func__, cnt);
		return -EINVAL;
	}

	grp = kzalloc((cnt / 3 + 1) * sizeof grp->gpios[0] + sizeof *grp,
		      GFP_KERNEL);

	if (!grp) {
		printk(KERN_ERR "%s: "
		       "failed to allocate memory for gpio group\n", __func__);
		rc = -ENOMEM;
		goto out;
	}
	grp->cnt=0;
	
	printk(KERN_ERR "%s: try to read %s  %d gpios\n", __func__, name, cnt/3);

	for (idx = 0; idx < cnt / 3; ++idx) {
		rc = trizeps7_reserved_read_gpio(np, name, idx, grp);
		if (rc < 0)
			goto out;
	}

	*gpio_group = grp;
	grp = NULL;

	rc = 0;

out:
	kfree(grp);

	return rc;
}


char *gpio_str( int gpio)
{
  static char gpiostring[32];
  if( gpio%32 >= 10 )
    sprintf( gpiostring, "%d.%2d", (gpio/32)+1, gpio%32);
  else
    sprintf( gpiostring, "%d.%d ", (gpio/32)+1, gpio%32);    
  return(gpiostring);  
}

static int kk_handle_single_reserved(struct device_node	*np)
{
	struct gpio_group	*grp = NULL;
	int			rc;
	size_t			i;
	int offset=0;	

	rc = trizeps7_reserved_read_gpio_group(np, "kk,os-gpios", &grp);
	if (rc < 0)
		goto out;

	if( read_alternative_data )
	  offset=320;

	for (i = 0; i < grp->cnt; ++i) {
		struct gpio_data const	*gpio = &grp->gpios[i];
		rc = gpio_request_one(gpio->gpio+offset, gpio->flags, gpio->label);
		
		if (rc < 0)
		{
		    printk(KERN_ERR "%s: "
			   "failed to request reserved gpio #%03u (%s): %d\n",
			   __func__, gpio->gpio+offset, gpio->label, rc);
		  
		    grp->cnt = i;
		    goto out;
		}else
		{
		    printk(KERN_ERR "%s: Requesting reserved gpio%s (%3d) %16s 0x%02x (%s):(%s)\n",
			   __func__,gpio_str(gpio->gpio+offset),gpio->gpio+offset,gpio->label, gpio->flags,
			   ((rc==0) ? "OK" : "failed"),
			   ((gpio->do_release)?"temporary":"keep allocated"));			
		}
		
		if (gpio->do_release)
		{
		    gpio_free(gpio->gpio+offset);
		}
	}

out:
	if (rc < 0) {
		for (i = grp ? grp->cnt : 0; i > 0; --i) {
			struct gpio_data const	*gpio = &grp->gpios[i-1];

			if (!gpio->do_release)
				gpio_free(gpio->gpio+offset);
		}
	}

	return rc;
}
  
// static
int kk_t7_handle_reserved(void)
{
	struct device_node	*np;
	int		        rc = 0;
	unsigned int		cnt = 0;

	if (!of_machine_is_compatible("kk,trizeps7") && !of_machine_is_compatible("kk,trizeps8"))
	{
	      printk(KERN_ERR "Not a kk,trizeps7 or kk,trizeps8 machine type\n");
	      return 0;
	}
	
	if( ! read_alternative_data )
	{
	  for_each_compatible_node(np, NULL, "kk,trizeps7-reserved") {
		rc = kk_handle_single_reserved(np);
		if (rc < 0)
			break;

		++cnt;
	  }
	}else
	{
	  for_each_compatible_node(np, NULL, "kk,trizeps8mcu-reserved") {
		rc = kk_handle_single_reserved(np);
		if (rc < 0)
			break;
		++cnt;
	  }
	}
	

	if (cnt == 0 && rc == 0) {
		printk(KERN_ERR "missing reserved nodes\n");
		rc = -ENOENT;
	}

	return rc;
}
#if 0
int __init kk_t7_handle_reserved_init(void)
{
  return(kk_t7_handle_reserved());  
}
#endif

int handle_reserved_gpio(void)
{
  read_alternative_data=0;
  return(kk_t7_handle_reserved());  
}


// device_initcall(kk_t7_handle_reserved_init);
// subsys_initcall(kk_t7_handle_reserved_init);
// arch_initcall_sync(kk_t7_handle_reserved);

int handle_reserved_mcu(void)
{
  read_alternative_data=1;
  return(kk_t7_handle_reserved());  
}


#define AIPS2_ARB_BASE_ADDR             0x02100000
#define ATZ2_BASE_ADDR                  AIPS2_ARB_BASE_ADDR
#define AIPS2_OFF_BASE_ADDR             (ATZ2_BASE_ADDR      + 0x80000)
#define OCOTP_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x3C000)
#define IMX_GPIO_NR(bank, nr)           (((bank) - 1) * 32 + (nr))

int ddr3l=UNDEFINED;
extern unsigned int system_rev;
extern unsigned int system_serial_low, system_serial_high;


TRIZEPS_INFO TrizepsBoardVersion = 
{
  .trizeps_module = 7,
  .trizeps_sodimm = 200,
  .trizeps_extcon  = 1,
  .trizeps_resetout_gpio    = IMX_GPIO_NR(7,  12),
  .trizeps_hw_board_version = BOARD_TRIZEPS7_V1R2,
  .trizeps_sw_board_version = BOARD_TRIZEPS7_V1R2,
  .trizeps_board_version_str= "TRIZEPS_VII_V1R2",
  .trizeps_unique_id[0]=0,
  .trizeps_unique_id[1]=0,
};

#if 0

PTRIZEPS_INFO get_trizeps_board_version(void)
{
	unsigned int value;
	PTRIZEPS_INFO pTr = &TrizepsBoardVersion;
	struct device_node *np;
	void __iomem *base;

	pTr->trizeps_module            =7; 
	pTr->trizeps_sodimm            =200;
	pTr->trizeps_btwlan            =UNDEFINED;
	pTr->trizeps_extcon            =UNDEFINED; 
	pTr->trizeps_ddr3l             =UNDEFINED; 
	pTr->trizeps_hw_board_version  =BOARD_TRIZEPS7_V1R2;
	pTr->trizeps_sw_board_version  =BOARD_TRIZEPS7_V1R2;
	pTr->trizeps_board_version_str ="TRIZEPS_VII_V1R?";
	pTr->trizeps_unique_id[0]      =UNDEFINED;
	pTr->trizeps_unique_id[1]      =UNDEFINED;
	pTr->trizeps_resetout_gpio    = UNDEFINED;

	if(cpu_is_imx6q())               // Default for mx6q -> No DDR3L
	  pTr->trizeps_ddr3l = 0;
	else
	  pTr->trizeps_ddr3l = 1;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return(pTr);
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		return(pTr);
	}

        pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	// Keith swap  5...0 -> 0....5
	value = readl_relaxed(base + 0x660);      // Board version;
	if( value & BOARD_TRIZEPS7_DDR3L_VALID)
	  pTr->trizeps_ddr3l=(value & BOARD_TRIZEPS7_HAS_DDR3L)?1:0;

	if( ddr3l == UNDEFINED )
	  ddr3l = pTr->trizeps_ddr3l;	

	// lowest 8 Bits for compatibility 
	switch (value&0xff)
	{
	default:
	case 0:
	  pTr->trizeps_board_version_str ="TRIZEPS_VII_V1R?";
	  pTr->trizeps_sw_board_version=BOARD_TRIZEPS7_V1R0;
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	  break;
	case 1:
	  pTr->trizeps_sw_board_version =BOARD_TRIZEPS7_V1R1;
	  pTr->trizeps_board_version_str="TRIZEPS_VII_V1R1";
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	  break;
	case 2:
	  pTr->trizeps_sw_board_version =BOARD_TRIZEPS7_V1R2;
	  pTr->trizeps_board_version_str="TRIZEPS_VII_V1R2";
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(1, 6);
	  break;
	case 3:
	  pTr->trizeps_sw_board_version =BOARD_TRIZEPS7_V1R3;
	  pTr->trizeps_board_version_str="TRIZEPS_VII_V1R3";
	  pTr->trizeps_resetout_gpio    = IMX_GPIO_NR(7, 12);
	  break;

	}
	system_rev=0x63000;
	// next 8 Bits for minor subreleases
	pTr->trizeps_hw_board_version=pTr->trizeps_sw_board_version+((value&0xff00)>>8);
	pTr->trizeps_unique_id[0]=system_serial_low =readl_relaxed(base + 0x410);
	pTr->trizeps_unique_id[1]=system_serial_high=readl_relaxed(base + 0x420);
	return(pTr);
}

void PrintTrizepsInfo(void)
{
     PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
     printk( KERN_ERR "********************* Trizeps7_Info ***************************************\n");
     if( cpu_is_imx6q() )
     printk("Trizeps_module   Version: %lu Dual/Quad\n", pTr->trizeps_module);
     else
     printk("Trizeps_module   Version: %lu Solo/Duallite\n",pTr->trizeps_module);
     printk("Trizeps Sodimm   Pins:    %lu\n",           pTr->trizeps_sodimm);
     printk("Trizeps DDR      Type:    %s",            ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
     if( ddr3l != pTr->trizeps_ddr3l )
       printk("overwrite cmdline with: %s", ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
     else
       printk("\n");     
     printk("Trizeps Version  String:  %s\n",          pTr->trizeps_board_version_str);
     printk("Trizeps HW Board Version: 0x%lx\n",       pTr->trizeps_hw_board_version);
     printk("Trizeps SW Board Version: 0x%lx\n",       pTr->trizeps_sw_board_version);
     printk("Trizeps nReset via Gpio   0x%lx\n",       pTr->trizeps_resetout_gpio);
     printk("Trizeps UniqueID:         0x%lx-%lx\n",   pTr->trizeps_unique_id[0],
	                                               pTr->trizeps_unique_id[1]);
     printk( KERN_ERR "***************************************************************************\n");
}

// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + +  @+01.08.2018 S&B HL
#include <linux/kernel.h>   
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define BUFSIZE  512
static struct proc_dir_entry *xEntry;							

static ssize_t iKKProcRead(struct file *file, char __user *ubuf,size_t count, loff_t *ppos) 
{
	char buf[BUFSIZE];
	int len=0;
    PTRIZEPS_INFO pTr= &TrizepsBoardVersion;

	if(*ppos > 0 || count < BUFSIZE)
		return 0;     
	if( cpu_is_imx6q() )
	len += sprintf(buf,		"Trizeps_module   Version = %lu Dual/Quad\n",    pTr->trizeps_module);
	else
	len += sprintf(buf,		"Trizeps_module   Version = %lu Solo/Duallite\n",pTr->trizeps_module);
	len += sprintf(buf + len,	"Trizeps Sodimm   Pins    = %lu\n",         pTr->trizeps_sodimm);
	len += sprintf(buf + len,	"Trizeps DDR      Type    = %s\n",          ((pTr->trizeps_ddr3l!=0)?"ddr3l":"ddr3"));
	len += sprintf(buf + len,	"Trizeps Version  String  = %s\n",          pTr->trizeps_board_version_str);
	len += sprintf(buf + len,	"Trizeps HW Board Version = 0x%lx\n",       pTr->trizeps_hw_board_version);
	len += sprintf(buf + len,	"Trizeps SW Board Version = 0x%lx\n",       pTr->trizeps_sw_board_version);
	len += sprintf(buf + len,	"Trizeps nReset via Gpio  = 0x%lx\n",       pTr->trizeps_resetout_gpio);
	len += sprintf(buf + len,	"Trizeps UniqueID         = 0x%lx-%lx\n",   pTr->trizeps_unique_id[0], pTr->trizeps_unique_id[1]);
	
	if(copy_to_user(ubuf,buf,len))
		return -EFAULT;
	*ppos = len;
	return len;
}

static struct file_operations tKKOps =	
{						
	.owner 	= THIS_MODULE,										
	.read 	= iKKProcRead,										
	.write 	= NULL,												
};																
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + +  @+01.08.2018 S&B HL


static int __init get_print_trizeps_version(void)
{
  get_trizeps_board_version();
  PrintTrizepsInfo();
  
  xEntry = proc_create("kk-trizeps7",0660,NULL,&tKKOps); 	// Create /proc/kk-trizeps7 entry	@+01.08.2018 S&B HL 
  return(0);  
}

// postcore_initcall(get_print_trizeps_version);
arch_initcall(get_print_trizeps_version);
#endif

static int __init kk_t7_set_info(void)
{
	uint32_t	rev;
	printk( KERN_ERR "***************************************************************************\n");
	printk( KERN_ERR "set_info\n");	
	if (!of_machine_is_compatible("kk,trizeps8") && !of_machine_is_compatible("kk,trizeps7"))
	{
	    printk( KERN_ERR "set_info NOT done\n");
	    printk( KERN_ERR "***************************************************************************\n");
	    return 0;	
	}
	
	if (!of_property_read_u32(NULL, "kk,trizeps7-revision", &rev))	system_rev = rev;
	if (!of_property_read_u32(NULL, "kk,trizeps8-revision", &rev))	system_rev = rev;

	printk( KERN_ERR "set_info done rev=0x%08x 0x%08x\n", system_rev, rev);
	printk( KERN_ERR "***************************************************************************\n");
	return 0;
}

// Output Modul e.g: 59A21.x2Hxx.Hxx

static char kuk_article_str[128];
static int __init kuk_article(char *s)
{
	strlcpy(kuk_article_str, s, sizeof(kuk_article_str));
	if(kuk_article_str[0]=='5' &&  kuk_article_str[1]=='9' ) TrizepsBoardVersion.trizeps_module=8;

	TrizepsBoardVersion.trizeps_temprange='C';
	TrizepsBoardVersion.trizeps_litevers=0;
	TrizepsBoardVersion.trizeps_sodimm=200;
	
	switch( kuk_article_str[2] )
	{
		case '0':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1800000000;
		  TrizepsBoardVersion.trizeps_litevers=1;
		  TrizepsBoardVersion.trizeps_numcores=1;		  
		  break;
		case '1':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1600000000;
		  TrizepsBoardVersion.trizeps_litevers=1;
		  TrizepsBoardVersion.trizeps_numcores=1;
		  TrizepsBoardVersion.trizeps_temprange='I';		  
		  break;
		case '2':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1800000000;		
		  TrizepsBoardVersion.trizeps_numcores=1;
		  break;
		case '3':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1600000000;		
		  TrizepsBoardVersion.trizeps_numcores=1;
		  TrizepsBoardVersion.trizeps_temprange='I';
		  break;
		case '4':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1800000000;		
		  TrizepsBoardVersion.trizeps_numcores=2;
		  TrizepsBoardVersion.trizeps_litevers=1;		  
		  break;
		case '5':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1600000000;		
		  TrizepsBoardVersion.trizeps_numcores=2;
		  TrizepsBoardVersion.trizeps_litevers=1;
		  TrizepsBoardVersion.trizeps_temprange='I';		  
		  break;
		case '6':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1800000000;		
		  TrizepsBoardVersion.trizeps_numcores=2;
		  break;
		case '7':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1600000000;		
		  TrizepsBoardVersion.trizeps_numcores=2;
		  TrizepsBoardVersion.trizeps_temprange='I';		  
		  break;
		case '8':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1800000000;		
		  TrizepsBoardVersion.trizeps_numcores=4;
		  TrizepsBoardVersion.trizeps_litevers=1;		  
		  break;
		case '9':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1600000000;		
		  TrizepsBoardVersion.trizeps_numcores=4;
		  TrizepsBoardVersion.trizeps_litevers=1;
		  TrizepsBoardVersion.trizeps_temprange='I';		  
		  break;
		case 'A':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1800000000;		
		  TrizepsBoardVersion.trizeps_numcores=4;
		  break;
		case 'B':
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1600000000;		
		  TrizepsBoardVersion.trizeps_numcores=4;
		  TrizepsBoardVersion.trizeps_temprange='I';		  
		  break;
		default:
		  TrizepsBoardVersion.trizeps_cpumaxfreq=1000000;		
		  TrizepsBoardVersion.trizeps_numcores=1;
		  TrizepsBoardVersion.trizeps_temprange='C';		  	  
	}
	TrizepsBoardVersion.trizeps_ramsize=1L*1024L*1024L*1024L;	
	switch( kuk_article_str[3] )
	{
		case '0':
		  TrizepsBoardVersion.trizeps_ramsize>>=1;
		  break;
		case '1':
		  // stay with 1GB
		  break;
		case '2':
		  TrizepsBoardVersion.trizeps_ramsize <<=1; // 2GB
		  break;
		case '4':
		  TrizepsBoardVersion.trizeps_ramsize <<=2; // 4GB
		  break;
		case '8':
		  TrizepsBoardVersion.trizeps_ramsize <<=3; // 8GB
		  break;
		default:
		  TrizepsBoardVersion.trizeps_ramsize >>=1; // 512MB
		  break;
	}
	switch( kuk_article_str[4] )
	{
		case '0':
		  TrizepsBoardVersion.trizeps_hw_board_version=0;
		  TrizepsBoardVersion.trizeps_board_version_str="TRIZEPS8MINI_V1R1";
		  break;
		case '1':
		  TrizepsBoardVersion.trizeps_hw_board_version=1;
		  TrizepsBoardVersion.trizeps_board_version_str="TRIZEPS8MINI_V1R2";
		  break;
		case '2':
		  TrizepsBoardVersion.trizeps_hw_board_version=2;
		  TrizepsBoardVersion.trizeps_board_version_str="TRIZEPS8MINI_V1R3";		
		  break;
		case '3':
		  TrizepsBoardVersion.trizeps_hw_board_version=3;
		  TrizepsBoardVersion.trizeps_board_version_str="TRIZEPS8MINI_V1R4";		
		  break;
	}

	TrizepsBoardVersion.trizeps_bootstoreemmc=0;
	switch( kuk_article_str[10] )
	{
		case '0':
		  TrizepsBoardVersion.trizeps_bootstore=KUK_BOOTSTORAGE_SDCARD;
		  break;
		case '1':
		  TrizepsBoardVersion.trizeps_bootstoreemmc=1;
		  TrizepsBoardVersion.trizeps_bootstore=KUK_BOOTSTORAGE_EMMC4GB;		  
		  break;
		case '2':
		  TrizepsBoardVersion.trizeps_bootstoreemmc=1;		  
		  TrizepsBoardVersion.trizeps_bootstore=KUK_BOOTSTORAGE_EMMC8GB;		  
		  break;
		case '3':
		  TrizepsBoardVersion.trizeps_bootstoreemmc=1;		  
		  TrizepsBoardVersion.trizeps_bootstore=KUK_BOOTSTORAGE_EMMC16GB;		  
		  break;
		case '4':
		  TrizepsBoardVersion.trizeps_bootstoreemmc=1;		  
		  TrizepsBoardVersion.trizeps_bootstore=KUK_BOOTSTORAGE_EMMC32GB;		  
		  break;
	}
	return 1;
}

__setup("kuk_article=", kuk_article);

#include <linux/kernel.h>   
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#define BUFSIZE  512
static struct proc_dir_entry *xEntry;							

static ssize_t iKKProcRead(struct file *file, char __user *ubuf,size_t count, loff_t *ppos) 
{
	char buf[BUFSIZE];
	int len=0;
	PTRIZEPS_INFO pTr= &TrizepsBoardVersion;

	if(*ppos > 0 || count < BUFSIZE)
		return 0;
	
	len += sprintf(buf,		"Trizeps_module   Version = %lu %d Cores %s\n",
			 pTr->trizeps_module,
			 pTr->trizeps_numcores,
		       ((pTr->trizeps_litevers) ? "Lite" : "") );

	len += sprintf(buf + len,	"Trizeps Sodimm   Pins    = %lu\n",         pTr->trizeps_sodimm);
	len += sprintf(buf + len,	"Trizeps Version  String  = %s\n",          pTr->trizeps_board_version_str);
	len += sprintf(buf + len,	"Trizeps HW Board Version = 0x%lx\n",       pTr->trizeps_hw_board_version);
	len += sprintf(buf + len,	"Trizeps SW Board Version = 0x%lx\n",       pTr->trizeps_sw_board_version);
	
	if(copy_to_user(ubuf,buf,len))
		return -EFAULT;
	*ppos = len;
	return len;
}

void PrintTrizepsInfo(void)
{
     PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
     printk( KERN_ERR "********************* Trizeps7_Info ***************************************\n");
     printk( KERN_ERR "Trizeps_module   Version = %lu %d Cores %s\n",
			 pTr->trizeps_module,
			 pTr->trizeps_numcores,
		       ((pTr->trizeps_litevers) ? "Lite" : "") );     

     printk("Trizeps Sodimm   Pins:    %lu\n",         pTr->trizeps_sodimm);
     printk("Trizeps Version  String:  %s\n",          pTr->trizeps_board_version_str);
     printk("Trizeps HW Board Version: 0x%lx\n",       pTr->trizeps_hw_board_version);
     printk("Trizeps SW Board Version: 0x%lx\n",       pTr->trizeps_sw_board_version);
     printk( KERN_ERR "***************************************************************************\n");
}

int TrizepsHasPCIeDisablePin(void)
{
  PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
  if( (pTr->trizeps_module == 8) && ( pTr->trizeps_hw_board_version < 2) )    return(1);      
  return(0);  
}

int TrizepsMMC0Has1V8Switch(void)
{
  PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
  if( (pTr->trizeps_module == 8) && ( pTr->trizeps_hw_board_version >= 2) )   return(1);      
  return(0);  
}

int TrizepsBootedFromeMMC(void)
{
  PTRIZEPS_INFO pTr= &TrizepsBoardVersion;
  if( (pTr->trizeps_module == 8) && pTr->trizeps_bootstoreemmc )     return(1);      
  return(0);  
}
  
static struct file_operations tKKOps =	
{						
	.owner 	= THIS_MODULE,										
	.read 	= iKKProcRead,										
	.write 	= NULL,												
};																
// + + + + + + + + + + + + + + + + + + + + + + + + + + + + + +  @+01.08.2018 S&B HL


static int __init get_print_trizeps_version(void)
{
  //  get_trizeps_board_version();
  PrintTrizepsInfo();
  
  xEntry = proc_create("kk-trizeps8mini",0660,NULL,&tKKOps); 	// Create /proc/kk-trizeps8 entry	@+01.08.2018 S&B HL 
  return(0);  
}

// postcore_initcall(get_print_trizeps_version);
arch_initcall(get_print_trizeps_version);


arch_initcall(kk_t7_set_info);
