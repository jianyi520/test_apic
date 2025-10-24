// 文件名: check_apic.c
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/msr.h>
#include <asm/processor.h>
#include <asm/cpufeature.h>
#include <linux/interrupt.h>
#include <asm/apic.h>
#include <asm/io.h>
#include <linux/cpu.h>
#include <linux/irq.h>

MODULE_LICENSE("GPL");

#define IA32_APIC_BASE_MSR 0x1B
#define X2APIC_VERSION_REG 0x803
#define X2APIC_LVT_TIMER   0x832
#define X2APIC_TIMER_INIT  0x838
#define X2APIC_TIMER_CUR   0x839  // Current Count Register

#define TIMER_VECTOR 0xEC  // 中断向量
#define IA32_PMC0    0x0C1   // PMC0 MSR

#define TIMER_INIT_COUNT   0x10000  // 初始计数值，可调整

static u64 pmc_value = 1;
static irqreturn_t apic_timer_irq(int irq, void *dev_id)
{
    pr_info(">>> APIC Timer Interrupt Handler Entered\n");//DEBUG
    // 读取 PMC0
    rdmsrl(IA32_PMC0, pmc_value);
    pr_info("中断读取PMU PMC0 Value: %llu\n", pmc_value);
    // 读取当前计数确认 DEBUG
    u64 current_count; //DEBUG
    rdmsrl(X2APIC_TIMER_CUR, current_count); //DEBUG
    pr_info("Current Count: 0x%llx\n", current_count);  //DEBUG
    apic_write(APIC_EOI, 0); //发送EOI
    pr_info("<<< EOI Sent, Interrupt Handler Exiting\n"); //DEBUG
    // 周期模式：中断触发后自动重载，不需手动操作
    return IRQ_HANDLED;
}


static int __init check_x2apic_timer_init(void)
{
    u32 eax, ebx, ecx, edx;
    u64 apic_base;
    u64 value;
    bool x2apic_supported;

    
    // 检测 CPU 是否支持 x2APIC
    cpuid(1, &eax, &ebx, &ecx, &edx);
    x2apic_supported = (ecx & (1 << 21)) ? true : false;
    if (!x2apic_supported) {
        pr_info("CPU does NOT support x2APIC\n");
        return -ENODEV;
    }
    pr_info("CPU supports x2APIC\n");

    // 读取 IA32_APIC_BASE MSR
    rdmsrl(IA32_APIC_BASE_MSR, apic_base);
    pr_info("IA32_APIC_BASE MSR: 0x%llx\n", apic_base);
    pr_info("APIC Global Enable (bit11): %llu\n", (apic_base >> 11) & 1);
    pr_info("x2APIC mode (EXTD bit10): %llu\n", (apic_base >> 10) & 1);

    // 读取 Local APIC Version
    rdmsrl(X2APIC_VERSION_REG, value);
    pr_info("x2APIC Version Register: 0x%llx\n", value);
    pr_info("Local APIC Version: 0x%llx\n", value & 0xFF);
    pr_info("Max LVT Entry: %llu\n", (value >> 16) & 0xFF);

    // 读取 LVT Timer 寄存器
    rdmsrl(X2APIC_LVT_TIMER, value);
    pr_info("LVT Timer Register (设置前): 0x%llx\n", value);    
    u8 vector;
    u8 mask;
    u8 mode;
    vector = value & 0xFF;          // 0-7 位
    mask   = (value >> 16) & 0x1;   // 16 位
    mode   = (value >> 17) & 0x3;   // 17-18 位

    pr_info("向量 (0-7 bits): 0x%x\n", vector);
    pr_info("Mask (16): 0x%x\n", mask);
    pr_info("模式 (17-18 bits): 0x%x\n", mode);


    // 清除原来的17-18位（Timer Mode）
    value &= ~(0x3ULL << 17);
    value &= ~(0xFFULL);       // 清除向量位
    // 设置为Periodic (01)
    value |= (0x1ULL << 17);
    value |= TIMER_VECTOR;      // 设置中断向量
    // 写回寄存器
    wrmsrl(X2APIC_LVT_TIMER, value);

    // 再次读取确认
    rdmsrl(X2APIC_LVT_TIMER, value);
    pr_info("LVT Timer Register (设置完周期模式后): 0x%llx\n", value);
    vector = value & 0xFF;          // 0-7 位
    mask   = (value >> 16) & 0x1;   // 16 位
    mode   = (value >> 17) & 0x3;   // 17-18 位

    pr_info("向量 修改后(0-7 bits): 0x%x\n", vector);
    pr_info("Mask 修改后(16): 0x%x\n", mask);
    pr_info("模式 修改后(17-18 bits): 0x%x\n", mode);
   
    // 写入 Initial Count 启动定时器
    wrmsrl(X2APIC_TIMER_INIT, TIMER_INIT_COUNT);
    rdmsrl(X2APIC_TIMER_INIT, value);
    pr_info("Initial Count Register: 0x%llx\n", value);
    
    // 读取 Current Count
    rdmsrl(X2APIC_TIMER_CUR, value);
    pr_info("Current Count Register: 0x%llx\n", value);
    
    // 注册中断处理函数  要改！！！
    int ret;
    ret = request_irq(TIMER_VECTOR, apic_timer_irq, IRQF_PERCPU | IRQF_NOBALANCING,
                      "my_lvt_timer", NULL);
    if (ret) {
        pr_err("[LVT Timer] Failed to request irq %d\n", TIMER_VECTOR);
        return ret;
    }
    
    wrmsrl(X2APIC_TIMER_INIT, TIMER_INIT_COUNT);
    pr_info("[LVT Timer] Handler registered for vector 0x%x\n", TIMER_VECTOR);

    return 0;
}

static void __exit check_x2apic_timer_exit(void)
{
    // 停止定时器
    wrmsrl(X2APIC_TIMER_INIT, 0);
    
    // 屏蔽定时器中断
    u64 value;
    rdmsrl(X2APIC_LVT_TIMER, value);
    value |= (1ULL << 16);  // 设置Mask位
    wrmsrl(X2APIC_LVT_TIMER, value);
    free_irq(TIMER_VECTOR, NULL);
    pr_info("x2APIC Timer module unloaded\n");
}

module_init(check_x2apic_timer_init);
module_exit(check_x2apic_timer_exit);

