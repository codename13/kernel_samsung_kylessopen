#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include "MMS100S_ISC_Updater_Customize.h"
#include "MMS100S_ISC_Updater.h"

#ifdef CONFIG_MACH_KYLE_I
#include <KYLE_I_R03_VA19.c>
#define TSP_SET_COMP	0x03
#define TSP_SET_VERSION	0x19
#else
#include "KYLE_CORE28_PR_31_VC05.c"
#endif
#include "KYLE_G1F_R00_VF22.c"

#define MFS_HEADER_		5
#define MFS_DATA_		20480
#define PACKET_			(MFS_HEADER_ + MFS_DATA_)

unsigned char g_write_buffer[PACKET_];
unsigned char IC_type;
bool module_type_check;
bool exception_condition = false;

#define ISC_CMD_ENTER_ISC						0x5F
#define ISC_CMD_ENTER_ISC_PARA1					0x01
#define ISC_CMD_ISC_ADDR						0xD5
#define ISC_CMD_ISC_STATUS_ADDR					0xD9

#define MODULE_COMPATIBILITY_ADDR	0x1C
#define SET_COMPATIBILITY_ADDR	0x4C09
#define FIRMWARE_VERSION_ADDR	0x1D
#define SET_VERSION_ADDR	0x4C08
#define PANEL_TYPE_ADDR					0x1F
#define CRC_DONE_ADDR						0xB1
#define CRC_ADDR							0xB2
#define READ_RETRY_CNT 3

/*
 * ISC Status Value
 */
#define ISC_STATUS_RET_MASS_ERASE_DONE			0X0C
#define ISC_STATUS_RET_MASS_ERASE_MODE			0X08

#define MFS_DEFAULT_SLAVE_ADDR	0x48

#define G1M_V34_CRC	0
#define G1F_V22_CRC	151
#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
#define G2_VA1_CRC	142
#endif

static eMFSRet_t enter_ISC_mode(void);
eMFSRet_t get_tsp_compatibility(unsigned char* tspcomp);
static eMFSRet_t check_module_compatibility(const unsigned char *_pBinary_Data);
static eMFSRet_t check_firmware_version(bool only_read, const unsigned char *_pBinary_Data);
eMFSRet_t check_panel_type(unsigned char* paneltype);
eMFSRet_t get_main_compatibility(const unsigned char *_pBinary_Data, unsigned char* maincomp);
eMFSRet_t MFS_ISC_force_update(void);

static int firmware_write(const unsigned char *_pBinary_Data);
static int firmware_verify(const unsigned char *_pBinary_Data);
static int mass_erase(void);
unsigned char* buf;

extern int melfas_fw_i2c_write(char *buf, int length);
extern int melfas_fw_i2c_read(u16 addr, u8 *value, u16 length);
extern int melfas_fw_i2c_read_without_addr(u8 *value, u16 length);
extern int melfas_fw_i2c_busrt_write(u8 *value, u16 length);

unsigned char TSP_PanelVersion, TSP_PhoneVersion;
eMFSRet_t MFS_ISC_update(void)
{
	extern void ts_power_enable(int en);
	eMFSRet_t ret;
	unsigned char paneltype, tspcomp, setcomp, redownload = 0;
	int i;
	MFS_I2C_set_slave_addr(mfs_i2c_slave_addr);

ENTRANCE:

	MFS_ms_delay(50);

#ifdef CONFIG_MACH_KYLE_I
	buf =& MELFAS_binary;
#else
	ret= check_panel_type(&paneltype);
	if (ret != MRET_SUCCESS && !redownload)
	{
		MFS_TSP_reboot();
		if (mass_erase() != MRET_SUCCESS)
			goto MFS_ISC_UPDATE_FINISH;

#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
		if (firmware_write(G2_BINARY) != MRET_SUCCESS)
			goto MFS_ISC_UPDATE_FINISH;			
#else
		if (firmware_write(G1F_BINARY) != MRET_SUCCESS)
			goto MFS_ISC_UPDATE_FINISH;
#endif

		MFS_TSP_reboot();
		if (enter_ISC_mode() != MRET_SUCCESS)
			goto MFS_ISC_UPDATE_FINISH;

#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
		if (firmware_verify(G2_BINARY) != MRET_SUCCESS)
			goto MFS_ISC_UPDATE_FINISH;
#else
		if (firmware_verify(G1F_BINARY) != MRET_SUCCESS)
			goto MFS_ISC_UPDATE_FINISH;
#endif

		MFS_TSP_reboot();
		redownload = 1;
		goto ENTRANCE;
	}
	else if (ret != MRET_SUCCESS)
	{
		goto MFS_ISC_UPDATE_FINISH;
	}

	if(paneltype == 0x4d)
	{
		buf = &G1M_BINARY;
		printk(KERN_ERR "<MELFAS> buf_G1M\n");
	}
	else if(paneltype == 0x46)
	{
		buf =& G1F_BINARY;
		printk(KERN_ERR "<MELFAS> buf_G1F\n");
	}
#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
	else if(paneltype == 0x58)
	{
		buf =& G2_BINARY;
		printk(KERN_ERR "<MELFAS> buf_G2\n");
		//goto MFS_ISC_UPDATE_FINISH;
	}
#endif
#endif

	// TSP HW Revision
	if( get_tsp_compatibility(&tspcomp) != MRET_SUCCESS)
		goto MFS_ISC_UPDATE_FINISH;


	printk(KERN_ERR "<MELFAS> paneltype=%c\n", paneltype);
	printk(KERN_ERR "<MELFAS> tspcomp=%x\n", tspcomp);
#if defined(CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2)
	if((paneltype == 0x4d && tspcomp > 0x0E) 
		|| (paneltype == 0x46 && ( tspcomp < 0x0F || tspcomp > 0x14 )) 
		|| (paneltype == 0x58 && ( tspcomp < 0x15 || tspcomp > 0x19 )) )
#else
	if((paneltype == 0x4d && tspcomp > 0x0E) || (paneltype == 0x46 && tspcomp <= 0x0E))
#endif
		goto START_DOWNLOAD;

	get_main_compatibility(buf, &setcomp);

#ifdef CONFIG_MACH_KYLE_I
	setcomp = TSP_SET_COMP;
	printk(KERN_ERR "<MELFAS> setcomp=%x\n", setcomp);
	printk(KERN_ERR "<MELFAS> tspcomp=%x\n", tspcomp);
	if(tspcomp != setcomp){
		goto MFS_ISC_UPDATE_FINISH;		
	}
#else
#if defined(CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2)
	if( paneltype == 0x58 )
	{
		setcomp = TSP_SET_COMP_G2;
		printk(KERN_ERR "<MELFAS> setcomp=%x\n", setcomp);
		printk(KERN_ERR "<MELFAS> tspcomp=%x\n", tspcomp);
		if(tspcomp != setcomp)
			goto MFS_ISC_UPDATE_FINISH;
	}
	else
#endif		
	{
		printk(KERN_ERR "<MELFAS> setcomp=%c\n", setcomp);
		printk(KERN_ERR "<MELFAS> tspcomp=%x\n", tspcomp);	
		if(tspcomp != (setcomp-55))
			goto MFS_ISC_UPDATE_FINISH;
	}
#endif

	if (check_firmware_version(0, buf) != MRET_SUCCESS)
		goto MFS_ISC_UPDATE_FINISH;

START_DOWNLOAD:
	if (mass_erase() != MRET_SUCCESS)
		goto MFS_ISC_UPDATE_FINISH;

	ret = firmware_write(buf);
	if (ret != MRET_SUCCESS)
		goto MFS_ISC_UPDATE_FINISH;

	MFS_TSP_reboot();
	printk(KERN_ERR "<MELFAS> TOUCH IC REBOOT!!!\n");
	
	if (enter_ISC_mode() != MRET_SUCCESS)
		goto MFS_ISC_UPDATE_FINISH;

	if (firmware_verify(buf) != MRET_SUCCESS)
		goto MFS_ISC_UPDATE_FINISH;

	printk(KERN_ERR "<MELFAS> FIRMWARE_UPDATE_FINISHED!!!\n\n");

MFS_ISC_UPDATE_FINISH:

	MFS_TSP_reboot();
	printk(KERN_ERR "<MELFAS> TOUCH IC REBOOT2!!!\n");
	check_firmware_version(1, buf);

	MFS_I2C_set_slave_addr(mfs_i2c_slave_addr);

	return ret;
}

eMFSRet_t check_panel_type(unsigned char* paneltype)
{
	eMFSRet_t ret;
	unsigned char read_buffer, cnt = 0;
	printk(KERN_ERR "<MELFAS> Check Panel Type\n");

	do
	{
		if (cnt++ > 3)
			return MRET_PANEL_TYPE_ERROR;
		
		if (!melfas_fw_i2c_read(0x1F, &read_buffer, 1))
			return MRET_I2C_ERROR;

		*paneltype = read_buffer;
		MFS_ms_delay(5);
#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
	}while(read_buffer != 0x46 && read_buffer != 0x4d && read_buffer != 0x58);
#else
	}while(read_buffer != 0x46 && read_buffer != 0x4d);
#endif

	printk(KERN_ERR "<MELFAS> read_buffer_value=%c\n", read_buffer);

	if (read_buffer == 0x4d)
		IC_type = 0x0D;
#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
	else if (read_buffer == 0x58)
		IC_type = 0x0C;		// 임의로 지정한 값입니다.
#endif
	else 
		IC_type = 0x0F;
	

		return MRET_SUCCESS;
}

eMFSRet_t get_tsp_compatibility(unsigned char* tspcomp)
{
	unsigned char write_buffer, read_buffer;
	unsigned char moduleComp;
	printk(KERN_ERR "<MELFAS> Get TSP Compatibility\n");

	if (!melfas_fw_i2c_read(MODULE_COMPATIBILITY_ADDR, &read_buffer, 1))
		return MRET_I2C_ERROR;

	*tspcomp = read_buffer;
	return MRET_SUCCESS;
}

eMFSRet_t get_main_compatibility(const unsigned char *_pBinary_Data, unsigned char* maincomp)
{
	printk(KERN_ERR"<MELFAS> Get main Compatibility\n");
	
	*maincomp = _pBinary_Data[SET_COMPATIBILITY_ADDR];
	
	return MRET_SUCCESS;
}

eMFSRet_t check_firmware_version(bool only_read, const unsigned char *_pBinary_Data)
{
	unsigned char write_buffer, read_buffer;
	unsigned char moduleVersion, setVersion;
	printk(KERN_ERR "<MELFAS> Check Firmware Version\n");
	melfas_fw_i2c_read(FIRMWARE_VERSION_ADDR, &read_buffer, 1);

	moduleVersion = read_buffer;
	setVersion = _pBinary_Data[SET_VERSION_ADDR];
#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
	if( IC_type == 0x0C )
		setVersion = TSP_SET_VERSION_G2;
#endif
	TSP_PanelVersion = moduleVersion;
	TSP_PhoneVersion = setVersion;
	printk(KERN_ERR "<MELFAS> TSP_PanelVersion=%2x\n", TSP_PanelVersion);
	printk(KERN_ERR "<MELFAS> TSP_PhoneVersion=%2x\n", TSP_PhoneVersion);
	if (only_read)
		return MRET_CHECK_VERSION_ERROR;
	
#ifdef CONFIG_MACH_KYLE_I
	setVersion = TSP_SET_VERSION;	/* temp. kylei CU white panel R03 */
	TSP_PhoneVersion = setVersion;
#endif

	if (moduleVersion < setVersion)
		return MRET_SUCCESS;
	else if (moduleVersion == setVersion)
	{
		printk(KERN_ERR "<MELFAS> Get CRC\n");	

		do
		{
		if (!melfas_fw_i2c_read(CRC_DONE_ADDR, &read_buffer, 1))
			return MRET_I2C_ERROR;
		} while (read_buffer != 0xAA);

		if (!melfas_fw_i2c_read(CRC_ADDR, &read_buffer, 1))
			return MRET_I2C_ERROR;

		printk(KERN_ERR "<MELFAS> IC_type=%x\n", IC_type);
		printk(KERN_ERR "<MELFAS> read_buffer=%d\n", read_buffer);
#ifdef CONFIG_TOUCHSCREEN_MELFAS_KYLE_G2
		if ((IC_type == 0x0D && read_buffer != G1M_V34_CRC) || (IC_type == 0x0F && read_buffer != G1F_V22_CRC) || (IC_type == 0x0C && read_buffer != G2_VA1_CRC))
#else
		if ((IC_type == 0x0D && read_buffer != G1M_V34_CRC) || (IC_type == 0x0F && read_buffer != G1F_V22_CRC))
#endif
			return MRET_SUCCESS;
		else
		{
			printk(KERN_ERR "<MELFAS> Same_firmware_version_CRC_pass_Do not need to download\n\n");
			return MRET_CHECK_VERSION_ERROR;
		}
	}
	else {
		printk(KERN_ERR "Do not need to download\n\n");
		printk(KERN_ERR "Module has a latest or same version\n");
		return MRET_CHECK_VERSION_ERROR;
	}
}

eMFSRet_t enter_ISC_mode(void)
{
	unsigned char write_buffer[2];
	printk(KERN_ERR "<MELFAS> ENTER_ISC_MODE\n\n");
	write_buffer[0] = ISC_CMD_ENTER_ISC;
	write_buffer[1] = ISC_CMD_ENTER_ISC_PARA1;
	if (!melfas_fw_i2c_write(write_buffer, 2))
		printk(KERN_ERR "<MELFAS> MMS100S Firmare is not exist!!!\n\n");
	MFS_ms_delay(50);
	return MRET_SUCCESS;
}

int firmware_write(const unsigned char *_pBinary_Data)
{
#define DATA_SIZE 1024
#define CLENGTH 4
	int i, lStartAddr = 0, lCnt = 0;

	printk(KERN_ERR "<MELFAS> firmware_write start !!!\n");
	while (lStartAddr*CLENGTH < 20*1024) {
		g_write_buffer[0] = ISC_CMD_ISC_ADDR;
		g_write_buffer[1] = (char)(lStartAddr & 0xFF);
		g_write_buffer[2] = (char)((lStartAddr>>8) & 0xFF);
		g_write_buffer[3] = g_write_buffer[4] = 0;

		for (i = 0; i < DATA_SIZE; i++)
			g_write_buffer[MFS_HEADER_ + i] = \
			_pBinary_Data[lStartAddr*CLENGTH + i];

		MFS_ms_delay(5);
		melfas_fw_i2c_busrt_write(g_write_buffer,
							MFS_HEADER_+DATA_SIZE);

		lCnt++;
		lStartAddr = DATA_SIZE*lCnt/CLENGTH;
	}

	MFS_ms_delay(5);
	return MRET_SUCCESS;
}

int firmware_verify(const unsigned char *_pBinary_Data)
{
#define DATA_SIZE 1024
#define CLENGTH 4

	int i, k = 0;
	unsigned char  write_buffer[MFS_HEADER_], read_buffer[DATA_SIZE];
	unsigned short int start_addr = 0;

	printk(KERN_ERR "<MELFAS> FIRMARE VERIFY...\n");
	MFS_ms_delay(5);
	while (start_addr * CLENGTH < MFS_DATA_) {
		write_buffer[0] = ISC_CMD_ISC_ADDR;
		write_buffer[1] = (char)((start_addr) & 0XFF);
		write_buffer[2] = 0x40 + (char)((start_addr>>8) & 0XFF);
		write_buffer[3] = write_buffer[4] = 0;


		if (!melfas_fw_i2c_write(write_buffer, MFS_HEADER_))
			return MRET_I2C_ERROR;

		MFS_ms_delay(5);
		if (!melfas_fw_i2c_read_without_addr(read_buffer, DATA_SIZE))
			return MRET_I2C_ERROR;

		for (i = 0; i < DATA_SIZE; i++)
			if (read_buffer[i] != _pBinary_Data[i + start_addr * CLENGTH]) {
				printk(KERN_ERR "<MELFAS> VERIFY Failed\n");
				printk(KERN_ERR \
						"<MELFAS> original : 0x%2x, buffer : 0x%2x, addr : %d \n",
						_pBinary_Data[i + start_addr*CLENGTH], read_buffer[i], i);
				return MRET_FIRMWARE_VERIFY_ERROR;
			}

		k++;
		start_addr = DATA_SIZE*k/CLENGTH;
		return MRET_SUCCESS;
	}
}

int mass_erase(void)
{
	int i = 0;
	const unsigned char mass_erase_cmd[MFS_HEADER_] = {
						ISC_CMD_ISC_ADDR, 0, 0xC1, 0, 0};
	unsigned char read_buffer[4] = { 0, };

	printk(KERN_ERR "<MELFAS> mass erase start\n\n");

	MFS_ms_delay(5);
	if (!melfas_fw_i2c_write(mass_erase_cmd, MFS_HEADER_))
		printk(KERN_ERR "<MELFAS> mass erase start write fail\n\n");
	MFS_ms_delay(5);
	while (read_buffer[2] != ISC_STATUS_RET_MASS_ERASE_DONE) {
		melfas_fw_i2c_write(mass_erase_cmd, MFS_HEADER_);
		MFS_ms_delay(1000);
		if (!melfas_fw_i2c_read(ISC_CMD_ISC_STATUS_ADDR, read_buffer, 4))

		MFS_ms_delay(1000);

		if (read_buffer[2] == ISC_STATUS_RET_MASS_ERASE_DONE) {
			printk(KERN_ERR "<MELFAS> Firmware Mass Erase done.\n");
			return MRET_SUCCESS;
		} else if (read_buffer[2] == ISC_STATUS_RET_MASS_ERASE_MODE)
			printk(KERN_ERR "<MELFAS> Firmware Mass Erase enter success!!!\n");

		MFS_ms_delay(1);
		if (i > 20)
			return MRET_MASS_ERASE_ERROR;
		i++;
	}
	return MRET_SUCCESS;
}
