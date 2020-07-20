#include <net/ax88796.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>


char asixmac[] = { 0xA0, 0xA1, 0xC0, 0x00, 0x01, 0x02 };
    
/*
 * Asix AX88796 Ethernet
 */
static struct ax_plat_data ipan_asix_platdata = {
	.flags		= AXFLG_MAC_FROMPLATFORM, /* defined later */
	.wordlength	= 2,
        .dcr_val        = 1,  // wordwise
	.gpoc_val       = 0x19,
        .rcr_val        = 4,  // Accept Broadcast
        .mac_addr       = asixmac,
};

static struct resource ipan_asix_resource[] = {
	[0] = {
	        .start = WEIM_ARB_BASE_ADDR,  // CS0_BASE_ADDR,
		.end   = WEIM_ARB_BASE_ADDR + (0x20 * 0x20) - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = gpio_to_irq(IPAN_ETH2_IRQ) | IRQF_TRIGGER_FALLING,
		.end   = gpio_to_irq(IPAN_ETH2_IRQ) | IRQF_TRIGGER_FALLING,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device asix_device = {
	.name		= "ax88796",
	.id		= 0,
	.num_resources 	= ARRAY_SIZE(ipan_asix_resource),
	.resource	= ipan_asix_resource,
	.dev		= {
		.platform_data = &ipan_asix_platdata
	}
};

#define ETHER_ADDR_LEN 6
static u8 ether_mac_addr[ETHER_ADDR_LEN];void __init ipan_init_eth(struct ax_plat_data *plat_data)
{
        int i;
        u64 serial = ((u64) 0xDEADBEEF << 32) | 0XBEEFDEAD;

        for (i = 0; i < ETHER_ADDR_LEN; i++) {
                ether_mac_addr[i] = serial & 0xff;
                serial >>= 8;
        }

        if (is_valid_ether_addr(ether_mac_addr)) {
                plat_data->flags |= AXFLG_MAC_FROMPLATFORM;
                plat_data->mac_addr = ether_mac_addr;
                printk(KERN_INFO "%s(): taking MAC from serial boot tag\n",
                        __func__);
        } else {
                plat_data->flags |= AXFLG_MAC_FROMDEV;
                printk(KERN_INFO "%s(): no valid serial boot tag found, "
                        "taking MAC from device\n", __func__);
        }
}

static int


static void init_asix_ethernet(void)
{
        printk( KERN_ERR "*************************** IPAN10 Init ASIX ETHERNET ********************\n");
	gpio_request(IPAN_ETH2_NRESET, "ASIX_nReset");
	gpio_direction_output(IPAN_ETH2_NRESET, 1);
	gpio_set_value(IPAN_ETH2_NRESET,        0);
	
	gpio_request(IPAN_ETH2_IRQ, "ASIX_ETH2_IRQ");
	gpio_direction_input(IPAN_ETH2_IRQ);
	gpio_free(IPAN_ETH2_IRQ);

	gpio_request(IPAN_ETH2_PME, "ASIX_ETH2_PME");  // Wakeup IRQ ...unused until now
	gpio_direction_input(IPAN_ETH2_PME);
	gpio_free(IPAN_ETH2_PME);

	msleep(1);	
	gpio_set_value(IPAN_ETH2_NRESET,        1);
	gpio_free(IPAN_ETH2_NRESET);

	//ipan_init_eth(&ipan_asix_platdata);
	platform_device_register(&asix_device);

}
