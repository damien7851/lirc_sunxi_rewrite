#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xaaf8fd3f, "module_layout" },
	{ 0x15692c87, "param_ops_int" },
	{ 0xcb6c2359, "lirc_dev_fop_close" },
	{ 0x32b3a249, "lirc_dev_fop_open" },
	{ 0xf7e9ef3a, "lirc_dev_fop_poll" },
	{ 0x4393f79, "lirc_dev_fop_write" },
	{ 0x18f7be82, "lirc_dev_fop_read" },
	{ 0xd95cd7e6, "no_llseek" },
	{ 0x4dca738, "lirc_unregister_driver" },
	{ 0x344a50ed, "gpio_release" },
	{ 0x2e1ca751, "clk_put" },
	{ 0x1e047854, "warn_slowpath_fmt" },
	{ 0xdb760f52, "__kfifo_free" },
	{ 0xa24ed5dd, "platform_driver_unregister" },
	{ 0x80320303, "platform_device_unregister" },
	{ 0xf20dabd8, "free_irq" },
	{ 0x66042fb5, "lirc_register_driver" },
	{ 0x82f11a24, "clk_enable" },
	{ 0x21c601c3, "clk_set_rate" },
	{ 0x3c30876b, "clk_get" },
	{ 0x383e8a27, "gpio_request_ex" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0xe527ffdd, "platform_device_put" },
	{ 0x354ddee2, "platform_device_add" },
	{ 0x4bf40c2f, "platform_device_alloc" },
	{ 0x4ec05cea, "platform_driver_register" },
	{ 0xc068440e, "__kfifo_alloc" },
	{ 0x4467122a, "__init_waitqueue_head" },
	{ 0x311b7963, "_raw_spin_unlock" },
	{ 0x72542c85, "__wake_up" },
	{ 0x27e1a049, "printk" },
	{ 0x74c97f9c, "_raw_spin_unlock_irqrestore" },
	{ 0xf23fcb99, "__kfifo_in" },
	{ 0xbd7083bc, "_raw_spin_lock_irqsave" },
	{ 0xc2161e33, "_raw_spin_lock" },
	{ 0x8a0c04b4, "lirc_dev_fop_ioctl" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=lirc_dev";


MODULE_INFO(srcversion, "250FABFDD257D1C17CE5294");
