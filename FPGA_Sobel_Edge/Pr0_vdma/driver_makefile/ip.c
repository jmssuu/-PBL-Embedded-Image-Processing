//=========================定義include檔及define檔=======================
#define IP_MAJOR		0     // 0: dynamic major number
#define IP_MINOR		0     // 0: dynamic minor number
#define IP_BASEADDRESS 0x43C00000     // start Video to AXI4s IP address
#define SIZE_OF_DEVICE 0x10000

#define VDMA_LITE_ADDR 0x43000000     // VDMA AXI-Lite address
#define VDMA_LITE_SIZE 0x10000

#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>                     //define file結構
#include <linux/types.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <asm/uaccess.h>                  //copy from/to user
#include <linux/ioctl.h>
#include <asm/io.h>
#include <linux/slab.h>

#define IRQ_INTR_NUM 61

#define VDMA_size 640*480    //(pxl)
//#define VDMA_data_width 8*4  //(bit) one unsigned long size = 4byte
#define VDMA_SIZE_OF_ADDR 0x4B000*4 //640*480*4 byte
//===============================定義裝置編號=============================
static int IP_major = IP_MAJOR;  //主編號（類型編號）
static int IP_minor = IP_MINOR;  //次編號（同類型不同裝置）
module_param(IP_major, int, 0664);//設定存取模式,module_param(,,存取模式)
module_param(IP_minor, int, 0664);//設定存取模式,module_param(,,存取模式)

//=======================================================================

//===============================define device structure=============================
static struct IP_Driver { //註冊字元裝置配置
    dev_t IP_devt;
    struct class *class;
    struct device *class_dev; //struct class_device *class_dev;
    struct cdev *cdev;
} IP;
//=======================================================================
volatile unsigned long *IP_wBASEADDRESS; //預先宣告變數除存記憶體映射位址

int value=0;
int result=0;

//=============Set VDMA Memory Space======================
volatile unsigned long VDMA_addr[VDMA_size]; 
unsigned long *VDMA_PHY_ADDR = 0;//預先宣告變數儲存記憶體映射位址 physical address
volatile unsigned long *VDMA_VIR_ADDR = 0;//預先宣告變數儲存記憶體映射位址 virtual address
//----------VDMA AXI-Lite-----------
volatile unsigned long *VDMA_LITE_VIR_ADDR = 0;//預先宣告變數儲存記憶體映射位址 virtual address
#define H_STRIDE            640
#define H_ACTIVE            640
#define V_ACTIVE            480

//#define VIDEO_BASEADDR0 DDR_BASEADDR + 0x2000000
//#define VIDEO_BASEADDR1 DDR_BASEADDR + 0x3000000
//#define VIDEO_BASEADDR2 DDR_BASEADDR + 0x4000000

//=============================file operation=========================
static int IP_open(struct inode *inode, struct file *file){
    return nonseekable_open(inode, file);
}

static int IP_release(struct inode *inode, struct file *file){

    return 0;
}

static int IP_read(struct file *file, char __user *buf, size_t size, loff_t *ppos){
    value = ioread32(IP_wBASEADDRESS);//+4);
    result = copy_to_user(buf, &value,size);
    return result;
}

static int IP_write(struct file *file,const char __user *buf, size_t size, loff_t *ppos){
    result = copy_from_user(&value, buf, size);
    iowrite32(value, IP_wBASEADDRESS);
    return result;
}

static long IP_ioctl(struct file *file, unsigned int cmnd, unsigned long arg){
    return 0;
}

static struct file_operations IP_fops = {
    .owner          = THIS_MODULE,
    .open           = IP_open,
    .release        = IP_release,
    .read           = IP_read,
    .write          = IP_write,
    .unlocked_ioctl = IP_ioctl
};



//==========================RX INIT HANDLER===========================
static irqreturn_t Rx_handler(int irq, void *dev_id){
    unsigned int Rx_buffer = 0;
    printk("Intrrupt push!!!!!\n");
    Rx_buffer = ioread32(IP_wBASEADDRESS);//+4);
    printk("Buffer value = %c \n",Rx_buffer);
    return IRQ_HANDLED;
}

//=================== VDMA Set =======================================

static int vdma_init(void) {

    printk("------Setting VDMA------\n");

    //------vdma memory space-----------------------------------
    //Get new VDMA physical address 
    VDMA_PHY_ADDR = kmalloc(sizeof(unsigned long)*VDMA_size, GFP_KERNEL);
    if (!VDMA_PHY_ADDR){
	printk("err:kmalloc VDMA_memory fail\n");
	return -ENODEV;
    }
    //VDMA_PHY_ADDR = (unsigned long *)virt_to_phys(&VDMA_addr);
    //virt_to_phys(volatile void *address);
    printk("VDMA memory physical address : %p\n",VDMA_PHY_ADDR); 
    //kfree(VDMA_PHY_ADDR);
    //holding VDMA physical address 
    if(!request_mem_region((resource_size_t)VDMA_PHY_ADDR, VDMA_SIZE_OF_ADDR,"VDMA_memory_IP"))
    {
        printk("err:Request VDMA_memory mem_region\n");
        return -ENODEV;
    }
    VDMA_VIR_ADDR = (unsigned long *)ioremap_nocache((resource_size_t)VDMA_PHY_ADDR, VDMA_SIZE_OF_ADDR);
    printk("VDMA memory virtual address : %p\n", VDMA_VIR_ADDR);

    //------vdma AXI-Lite---------------------------------------
    //holding VDMA physical address 
    if(!request_mem_region(VDMA_LITE_ADDR, VDMA_LITE_SIZE,"VDMA_AXI_Lite_IP"))
    {
        printk("err:Request VDMA_AXI-LITE mem_region\n");
        return -ENODEV;
    }
    VDMA_LITE_VIR_ADDR = (unsigned long *)ioremap_nocache(VDMA_LITE_ADDR, VDMA_LITE_SIZE);
    printk("VDMA AXI-Lite virtual address : %p\n", VDMA_LITE_VIR_ADDR);

    //set vdma write
//    iowrite32(0x00000001, VDMA_LITE_VIR_ADDR + 0x030/4);			// enable circular mode
//    mb();
//    iowrite32((resource_size_t)VDMA_PHY_ADDR , VDMA_LITE_VIR_ADDR + 0x0AC/4);	// start address
//    mb();
    //iowrite32((resource_size_t)VDMA_PHY_ADDR + 0x1000000 , VDMA_LITE_VIR_ADDR + 0x0B0/4);	// start address
    //iowrite32((resource_size_t)VDMA_PHY_ADDR + 0x2000000 , VDMA_LITE_VIR_ADDR + 0x0B4/4);	// start address
//    iowrite32((H_STRIDE*4) , VDMA_LITE_VIR_ADDR + 0x0A8/4);		// h offset (640 * 4) bytes
//    mb();
//    iowrite32((H_ACTIVE*4) , VDMA_LITE_VIR_ADDR + 0x0A4/4);		// h size (640 * 4) bytes
//    mb();
//    iowrite32(V_ACTIVE , VDMA_LITE_VIR_ADDR + 0x0A0/4);			// v size (480)
//    mb();

    //set vdma read
    iowrite32(0x00000001 , VDMA_LITE_VIR_ADDR);// + 0x000);			// enable circular mode
    mb();
    iowrite32((resource_size_t)VDMA_VIR_ADDR , VDMA_LITE_VIR_ADDR + 23);//0x05c/4);	// start address
    mb();
    //iowrite32((resource_size_t)VDMA_PHY_ADDR + 0x1000000 , VDMA_LITE_VIR_ADDR + 0x060/4);	// start address
    //iowrite32((resource_size_t)VDMA_PHY_ADDR + 0x2000000 , VDMA_LITE_VIR_ADDR + 0x064/4);	// start address
    iowrite32((H_STRIDE*4) , VDMA_LITE_VIR_ADDR + 22);//0x058/4);		// h offset (640 * 4) bytes
    mb();
    iowrite32((H_ACTIVE*4) , VDMA_LITE_VIR_ADDR + 21);//0x054/4);		// h size (640 * 4) bytes
    mb();
    iowrite32(V_ACTIVE , VDMA_LITE_VIR_ADDR + 20);//0x050/4);			// v size (480)
    mb();

    //-----chack-------------------------------------
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x030/4));//48//0x030
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x0AC/4));//172//0x0AC
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x0A8/4));//168//0x0A8
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x0A4/4));//164//0x0A4
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x0A0/4));//160//0x0A0
    rmb();

    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x000/4));//0//0x000
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x05c/4));//92//0x05c
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x058/4));//88//0x058
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x054/4));//84//0x054
    rmb();
    printk("%p\n",(unsigned long *)ioread32(VDMA_LITE_VIR_ADDR + 0x050/4));//80//0x050
    rmb();

    printk("------Set VDMA finish------\n\n");
    return 0;
    
}
static int vdma_remove(void) {
    //iounmap((void *)VDMA_VIR_ADDR);//取消記憶體映射
    release_mem_region((resource_size_t)VDMA_PHY_ADDR, VDMA_SIZE_OF_ADDR);//取消物理記憶體位址保留
    //==============ioremap_nocache==================
    iounmap((void *)VDMA_LITE_VIR_ADDR);//取消記憶體映射
    release_mem_region(VDMA_LITE_ADDR, VDMA_LITE_SIZE);//取消物理記憶體位址保留
    return 0;
}
//==========================cdev init()     =============================
static int IP_setup_cdev(struct IP_Driver *IP_p){

    int ret, err;

    IP_p->IP_devt = MKDEV(IP_major,IP_minor); //向 kernel 取出 major/minor number

    if(IP_major){
	//register_chrdev_region 靜態取得 major number
        ret = register_chrdev_region(IP_p->IP_devt, 1, "IP-Driver");
    }else{
	//alloc_chrdev_region 動態取得 major number
        ret = alloc_chrdev_region(&IP_p->IP_devt, IP_minor, 1, "IP-Driver");
        IP_major = MAJOR(IP_p->IP_devt);//向 kernel 取出 major number
        IP_minor = MINOR(IP_p->IP_devt);//向 kernel 取出 minor number
    }
    if(ret <0)
        return ret;

    //--在「/sys/class/IP-Driver/IP-Driver/dev」新建立驅動程式資訊與規則檔---
    IP_p->class = class_create(THIS_MODULE, "IP-Driver");//登記 class , 讓驅動程式支援 udev
    if(IS_ERR(IP_p->class)){	//透過 IS_ERR() 來判斷class_create函式呼叫的成功與否
        printk("IP_setup_cdev: Can't create IP Driver class!\n");
        ret = PTR_ERR(IP_p->class);
        goto error1;
    }
    
    IP_p->cdev = cdev_alloc();//配置cdev的函式
    if(NULL == IP_p->cdev){
        printk("IP_setup_cdev: Can't alloc IP Driver cdev!\n");
        ret = -ENOMEM;
        goto error2;
    }
    
    IP_p->cdev->owner = THIS_MODULE;
    IP_p->cdev->ops = &IP_fops;

    err = cdev_add(IP_p->cdev, IP_p->IP_devt, 1);//向 kernel 登記驅動程式
    if(err){
        printk("IP_setup_cdev: Can't add IP cdev to system!\n");
        ret = -EAGAIN;
        goto error2;
    }

    //--建立「/sys/class/IP-Driver/IP-Driver」裝置名稱---
    IP_p->class_dev = device_create(IP_p->class, NULL, IP_p->IP_devt, NULL, "IP-Driver");

    if(IS_ERR(IP_p->class_dev)){ //透過 IS_ERR() 來判斷device_create函式呼叫的成功與否
        printk("IP_setup_cdev: Can't create IP class_dev to system!\n");
        ret = PTR_ERR(IP_p->class_dev);
        goto error3;
    }
    printk("IP-Driver_class_dev info: IP-Driver (%d:%d)\n",MAJOR(IP_p->IP_devt), MINOR(IP_p->IP_devt));
    
    //================request mem region======================
    //保留位址範圍,(記憶體位址,byte單位的範圍大小,引數裝置名稱)
    if(!request_mem_region(IP_BASEADDRESS, SIZE_OF_DEVICE*4,"IP"))
    {
        printk("err:Request_mem_region\n");
        return -ENODEV;
    }
    //===========ioremap_nocache===========//記憶體映射,將實體address交給linux去分配
    IP_wBASEADDRESS = (unsigned long *)ioremap_nocache(IP_BASEADDRESS, SIZE_OF_DEVICE*4);

    printk("IP > IP_ioport_write    : %p\n", IP_wBASEADDRESS);

    return 0;

    error3:
        cdev_del(IP_p->cdev);
    error2:
        class_destroy(IP_p->class);
    error1:
        unregister_chrdev_region(IP_p->IP_devt, 1);
        return ret;
}
//=======================================================================
static void IP_remove_cdev(struct IP_Driver *IP_p){

    device_unregister(IP_p->class_dev);
    cdev_del(IP_p->cdev);
    class_destroy(IP_p->class);
    unregister_chrdev_region(IP_p->IP_devt, 1);
    //==============ioremap_nocache==================
    iounmap((void *)IP_wBASEADDRESS);//取消記憶體映射

    release_mem_region(IP_BASEADDRESS, SIZE_OF_DEVICE*4);//取消物理記憶體位址保留

}
//=======================================================================

static int IP_init(void){
    int ret=0;

    printk(KERN_ALERT "~~~hello~~~\n");
    
    IP_setup_cdev(&IP);
    
    ret = request_irq(61,Rx_handler,0,"IP-Rx_ISR~",THIS_MODULE);
        if(ret < 0)
            pr_err("%s\n", "request_irq failed");

    printk("Initional IRQ finish!!!!!!\n");

    //iowrite32(ioread32(IP_wBASEADDRESS),IP_wBASEADDRESS);
    iowrite32(0x00,IP_wBASEADDRESS); //write data to memory(value , address)
    mb();
    printk("Initional device finish\n\n");  

    vdma_init();  //setting VDMA

    iowrite32(0x00000001,IP_wBASEADDRESS); //write data to memory(value , address)
    mb();
    iowrite32(0x00000011,IP_wBASEADDRESS+2); //write data to memory(value , address)
    mb();
    printk("%p\n",(unsigned long *)ioread32(IP_wBASEADDRESS+1));//48//0x030
    rmb();
    printk("Start VDMA!!\n\n");  

    return ret;
}

static void IP_exit(void){
    printk(KERN_ALERT "Close Modul~~~~~ \n");
    IP_remove_cdev(&IP);
    vdma_remove();
    kfree(VDMA_PHY_ADDR);
    free_irq(61,THIS_MODULE);
}

module_init(IP_init);             /*模組安裝之啟動函式*/
module_exit(IP_exit);             /*模組卸載之啟動函式*/

MODULE_DESCRIPTION("IP Driver");  /*此程式介紹與描述*/
MODULE_LICENSE("GPL");            /*程式 License*/
