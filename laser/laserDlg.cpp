
// laserDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "laser.h"
#include "laserDlg.h"
#include "afxdialogex.h"
#include "iostream"
#include "Baocun.h"
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <math.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框
enum {
  Timeout = 800,
  LineLength = 16 + 64 + 1 + 1 + 1,
};

typedef struct {
  enum {
    MODL = 0,		//!< Sensor Model
    DMIN,			//!< Min detection range [mm]
    DMAX,			//!< Man detection range [mm]
    ARES,			//!< Angular resolution (division of 360degree)
    AMIN,			//!< Min Measurement step
    AMAX,			//!< Max Measurement step
    AFRT,			//!< Front Step 
    SCAN,			//!< Standard scan speed
  };
  std::string model;		//!< Obtained Sensor Model,  MODL
  long distance_min;		//!< Obtained DMIN 
  long distance_max;		//!< Obtained DMAX 
  int area_total;		//!< Obtained ARES 
  int area_min;			//!< Obtained AMIN 
  int area_max;			//!< Obtained AMAX 
  int area_front;		//!< Obtained AFRT 
  int scan_rpm;			//!< Obtained SCAN 

  int first;			//!< Scan Start position
  int last;			//!< Scan end position
  int max_size;			//!< Max. size of data
  long last_timestamp; //!< Time stamp of the last obtained data
} urg_state_t;

static HANDLE HComm = INVALID_HANDLE_VALUE;
static char* ErrorMessage = "no error.";
//#define RS232 "MAXON_RS232"
//#define CANOpen "CANopen"
static long date[1000];/////////////////////////
 long px1=20;
 long py1=20; 
 //long dSn=10;
/////////////////////////激光传感器///////////////////////////////////////////
static int com_connect(const char* device, long baudrate) 
{
	
	char adjust_device[16];
	sprintf_s(adjust_device, "\\\\.\\%s", device);
	HComm = CreateFile(adjust_device, GENERIC_READ | GENERIC_WRITE, 0,
		NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	
	if (HComm == INVALID_HANDLE_VALUE) 
	{
		return -1;
	}
	
	// Baud Rate setting
	// !!! Not done
	
	return true;
}

static void com_disconnect(void)
{
	if (HComm != INVALID_HANDLE_VALUE) 
	{
		CloseHandle(HComm);
		HComm = INVALID_HANDLE_VALUE;
	}
}


static int com_send(const char* data, int size)
{
	DWORD n;
	WriteFile(HComm, data, size, &n, NULL);
	return n;
}


static int com_recv(char* data, int max_size, int timeout) 
{
	int filled = 0;
	int readable_size = 0;
	
	// Reading of Concerned data.
	do {
		DWORD dwErrors;
		COMSTAT ComStat;
		ClearCommError(HComm, &dwErrors, &ComStat);
		readable_size = (int)ComStat.cbInQue;
		int read_n = (readable_size > max_size) ? max_size : readable_size;
		
		DWORD n;
		ReadFile(HComm, &data[filled], read_n, &n, NULL);
		filled += n;
		readable_size -= n;
		
		if (filled >= max_size) 
		{
			return filled;
		}
	} while (readable_size > 0);	
	if (timeout > 0)
	{
		// Read data using time out
		COMMTIMEOUTS pcto;
		GetCommTimeouts(HComm, &pcto);
		pcto.ReadIntervalTimeout = 0;
		pcto.ReadTotalTimeoutMultiplier = 0;
		pcto.ReadTotalTimeoutConstant = timeout;
		SetCommTimeouts(HComm, &pcto);
		//Read data one charaters each
		DWORD n;
		while (1) 
		{
			ReadFile(HComm, &data[filled], 1, &n, NULL);
			if (n < 1) 
			{
				return filled;
			}
			filled += n;
			if (filled >= max_size)
			{
				return filled;
			}
		}
	}
	return filled;
}


// Send data(Commands) to URG 
static int urg_sendTag(const char* tag) 
{	
	char send_message[LineLength];
	sprintf_s(send_message, "%s\n", tag);
	int send_size = strlen(send_message);
	com_send(send_message, send_size);	
	return send_size;
}


// Read data (Reply) from URG until the termination 
static int urg_readLine(char *buffer) 
{	
	int i;
	for (i = 0; i < LineLength -1; ++i) 
	{
		char recv_ch;
		int n = com_recv(&recv_ch, 1, Timeout);
		if (n <= 0) 
		{
			if (i == 0) 
			{
				return -1;		// timeout
			}
			break;
		}
		if ((recv_ch == '\r') || (recv_ch == '\n'))
		{
			break;
		}
		buffer[i] = recv_ch;
	}
	buffer[i] = '\0';	
	return i;
}


// Send data (Commands) to URG and wait for reply
static int urg_sendMessage(const char* command, int timeout, int* recv_n) 
{
	int send_size = urg_sendTag(command);
	int recv_size = send_size + 2 + 1 + 2;
	char buffer[LineLength];	
	int n = com_recv(buffer, recv_size, timeout);
	*recv_n = n;	
	if (n < recv_size) 
	{
		// When received size not matched
		return -1;
	}	
	if (strncmp(buffer, command, send_size -1))
	{
		// When command not matched
		return -1;
	}	
	// !!! If possible do calculate check-sum to verify data	
	// Convert the received data to Hex and return data.
	char reply_str[3] = "00";
	reply_str[0] = buffer[send_size];
	reply_str[1] = buffer[send_size + 1];
	return strtol(reply_str, NULL, 16);
}


// Read URG parameters
static int urg_getParameters(urg_state_t* state) 
{
	// Parameter read and prcessing (use)
	urg_sendTag("PP");
	char buffer[LineLength];
	int line_index = 0;
	enum {
		    TagReply = 0,
			DataReply,
			Other,
	};
	int line_length;
	for (; (line_length = urg_readLine(buffer)) > 0; ++line_index) 
	{		
		if (line_index == Other + urg_state_t::MODL)
		{
			buffer[line_length - 2] = '\0';
			state->model = &buffer[5];
		} 
		else if (line_index == Other + urg_state_t::DMIN) 
		{
			state->distance_min = atoi(&buffer[5]);
		} 
		else if (line_index == Other + urg_state_t::DMAX) 
		{
			state->distance_max = atoi(&buffer[5]);
		} 
		else if (line_index == Other + urg_state_t::ARES) 
		{
			state->area_total = atoi(&buffer[5]);
		}
		else if (line_index == Other + urg_state_t::AMIN) 
		{
			state->area_min = atoi(&buffer[5]);
			state->first = state->area_min;
		}
		else if (line_index == Other + urg_state_t::AMAX) 
		{
			state->area_max = atoi(&buffer[5]);
			state->last = state->area_max;
		} 
		else if (line_index == Other + urg_state_t::AFRT)
		{
			state->area_front = atoi(&buffer[5]);
		}
		else if (line_index == Other + urg_state_t::SCAN)
		{
			state->scan_rpm = atoi(&buffer[5]);
		}
	}
	
	if (line_index <= Other + urg_state_t::SCAN)
	{
		return -1;
	}
	// Caluclate size of the data if necessary
	state->max_size = state->area_max +1;
	
	return 0;
}


// Process to connect with URG 
static int urg_connect(urg_state_t* state,
					   const char* port, const long baudrate)
{	
	if (com_connect(port, baudrate) < 0) 
	{
		ErrorMessage = "Cannot connect COM device.";
		return -1;
	}	
	const long try_baudrate[] = { 19200, 115200, 38400 };
	for (size_t i = 0; i < sizeof(try_baudrate)/sizeof(try_baudrate[0]); ++i) 
	{		
		// Change baud rate to detect the compatible baud rate with sensor
		// !!! com_changeBaudrate(try_baudrate[i]);		
		// Change to SCIP2.0 mode
		int recv_n = 0;
		urg_sendMessage("SCIP2.0", Timeout, &recv_n);
		if (recv_n <= 0) 
		{
			// If there is no reply it is considered as baud rate incompatibility, try with different baud rate
			continue;
		}		
		// Change the baud rate if not match the setting
		if (try_baudrate[i] != baudrate) 
		{
			// !!! urg_changeBaudrate(baudrate);
			// !!! The change will be implemented after 1 scan.
			// !!! Thus, Host or PC should wait to send the command			
			// !!! com_changeBaudrate(baudrate);
		}		
		// Read parameter
		if (urg_getParameters(state) < 0) 
		{
			ErrorMessage = "PP command fail.";
			return -1;
		}
		state->last_timestamp = 0;		
		// URG is detected
		return 0;
	}	
	// URG is not detected
	return -1;
}

// Disconnect URG 
static void urg_disconnect(void)
{
	com_disconnect();
}

// Data read using GD-Command
static int urg_captureByGD(const urg_state_t* state)
{
	char send_message[LineLength];
	sprintf_s(send_message, "GD%04d%04d%02d", state->first, state->last, 0);
	return urg_sendTag(send_message);
}


// Data read using MD-Command
static int urg_captureByMD(const urg_state_t* state, int capture_times) 
{	
	char send_message[LineLength];
	sprintf_s(send_message, "MD%04d%04d%02d%01d%02d",
		state->first, state->last, 0, 0, capture_times);
	return urg_sendTag(send_message);
}


// Decode 6 bit data from URG 
static long urg_decode(const char* data, int data_byte)
{
	long value = 0;
	for (int i = 0; i < data_byte; ++i) 
	{
		value <<= 6;
		value &= ~0x3f;
		value |= data[i] - 0x30;
	}
	return value;
}

// Receive URG distance data 
static int urg_addRecvData(const char buffer[], long data[], int* filled) 
{	
	static int remain_byte = 0;
	static char remain_data[3];
	const int data_byte = 3;
	const char* pre_p = buffer;
	const char* p = pre_p;	
	if (remain_byte > 0) 
	{
		memmove(&remain_data[remain_byte], buffer, data_byte - remain_byte);
		data[*filled] = urg_decode(remain_data, data_byte);
		++(*filled);
		pre_p = &buffer[data_byte - remain_byte];
		p = pre_p;
		remain_byte = 0;
	}	
	do {
		++p;
		if ((p - pre_p) >= static_cast<int>(data_byte)) 
		{
			data[*filled] = urg_decode(pre_p, data_byte);
			++(*filled);
			pre_p = p;
		}
	} while (*p != '\0');
	remain_byte = p - pre_p;
	memmove(remain_data, pre_p, remain_byte);
	return 0;
}

// Receive URG data
static int urg_receiveData(urg_state_t* state, long data[], size_t max_size) 
{
	int filled = 0;
	// Fill the positions upto first or min by 19 (non-measurement range)
	for (int j = state->first -1; j >= 0; --j) 
	{
		data[filled++] = 19;
	}	
	char message_type = 'M';
	char buffer[LineLength];
	int line_length;
	for (int i = 0;  (line_length = urg_readLine(buffer))>= 0;++i) 
	{
		// Verify the checksum
		if ((i >= 6) && (line_length == 0)) 
		{		
			// End of data receive
			for (size_t i = filled; i < max_size; ++i) 
			{
				// Fill the position upto data end by 19 (non-measurement range)
				data[filled++] = 19;
			}
			return filled;			
		}
		else if (i == 0) 
		{
			// Judge the message (Command) by first letter of receive data
			if ((buffer[0] != 'M') && (buffer[0] != 'G')) 
			{
				return -1;
			}
			message_type = buffer[0];			
		} 
		else if (! strncmp(buffer, "99b", 3)) 
		{
			// Detect "99b" and assume [time-stamp] and [data] to follow
			i = 4;
		} 
		else if ((i == 1) && (message_type == 'G')) 
		{
			i = 4;
		} 
		else if (i == 4) 
		{
			// "99b" Fixed
			if (strncmp(buffer, "99b", 3)) 
			{
				return -1;
			}
		} 
		else if (i == 5) 
		{
			state->last_timestamp = urg_decode(buffer, 4);
		} 
		else if (i >= 6) 
		{
			// Received Data
			if (line_length > (64 + 1)) 
			{
				line_length = (64 + 1);
			}
			buffer[line_length -1] = '\0';
			int ret = urg_addRecvData(buffer, data, &filled);
			if (ret < 0)
			{
				return ret;
			}
		}
	}
	return -1;
}
/////////////////////////////////////////////////////////////////////////////////////////////
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// ClaserDlg 对话框

//////////////////////////////////////////////////////////////////////////////////


ClaserDlg::ClaserDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(ClaserDlg::IDD, pParent)
{
	m_rece = 0;
	//  m_mid = 0;
	//  m_right = 0;
	//  m_left = 0;
	m_width = 0;
	m_height = 0;
	m_gox = 1000;
	m_goy = 1000;
	m_inx = 20;
	m_iny = 20;
	m_tx = 0;
	m_ty = 0;
	m_corner = 0.0;
	m_distance = 0;
	//////////////////////////////////////////////////////////////////////////
	//m_strDeviceName="EPOS";            ///设备名称
	//m_strProtocalStackName="CANOpen";  ///接口名称
	//m_strInterfaceName="IXXAT_USB-to-CAN compact 0";   ///通信方式
	//m_strPortName="CAN0";              ///端口名称
	//m_wLeftMotorID=3;
	//m_wRightMotorID=1;
	//m_dErrorCode=10020;
	//MotorEnabled=FALSE;
	//m_lVelocitySpeed=1000;	
	//m_lMotorForwardVelocityMust=1000;
	//m_lLeftMotorVelocityMust=1000;
	//m_lRightMotorVelocityMust=1000;
	////////////////////////////////////////////////////////////////////////////
	m_HMRHangxj = 0.0f;
	m_HMRHenggj = 0.0f;
	m_HMRFuyj = 0.0f;
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	m_1 = 0;
	m_2 = 0;
	m_3 = 0;
	m_4 = 0;
	m_5 = 0;
	m_6 = 0;
	m_7 = 0;
	m_8 = 0;
	m_9 = 0;
}

void ClaserDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	/////////////////////////激光雷达几个数据/////////////////////////////////////
	//DDX_Text(pDX, IDC_MID, m_mid);                     
	//DDX_Text(pDX, IDC_RIGHT, m_right);
	//DDX_Text(pDX, IDC_LEFT, m_left);
	/////////////////////////////////////////////////////////////////
	/*DDX_Text(pDX, IDC_GOX, m_gox);
	DDX_Text(pDX, IDC_GOY, m_goy);
	DDX_Text(pDX, IDC_INX, m_inx);
	DDX_Text(pDX, IDC_INY, m_iny);
	DDX_Text(pDX, IDC_TARGETX, m_tx);
	DDX_Text(pDX, IDC_TARGETY, m_ty);
	DDX_Text(pDX, IDC_CORNER, m_corner);
	DDX_Text(pDX, IDC_DISTANCE, m_distance);
	DDX_Text(pDX, IDC_EDIT_HMR_Hangxj, m_HMRHangxj);
	DDX_Text(pDX, IDC_EDIT_HMR_Henggj, m_HMRHenggj);
	DDX_Text(pDX, IDC_EDIT_HMR_Fuyj, m_HMRFuyj);
	*/
	//  DDX_Control(pDX, IDC_EDIT1, m_1);
	DDX_Text(pDX, IDC_EDIT1, m_1);
	DDX_Text(pDX, IDC_EDIT2, m_2);
	DDX_Text(pDX, IDC_EDIT3, m_3);
	DDX_Text(pDX, IDC_EDIT4, m_4);
	DDX_Text(pDX, IDC_EDIT5, m_5);
	DDX_Text(pDX, IDC_EDIT6, m_6);
	DDX_Text(pDX, IDC_EDIT7, m_7);
	DDX_Text(pDX, IDC_EDIT8, m_8);
	DDX_Text(pDX, IDC_EDIT9, m_9);
}

BEGIN_MESSAGE_MAP(ClaserDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	///////////////////////////////////////////////////////////////////////
	                      
	ON_BN_CLICKED(IDC_SHOW, OnShow)
	
	ON_BN_CLICKED(IDC_StartScan, OnStartScan)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_StopScan, OnStopScan)
	/////////////////////////////////////////////////////////////////////////
END_MESSAGE_MAP()


// ClaserDlg 消息处理程序

BOOL ClaserDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	pButtonOne=(CButton*) GetDlgItem(IDC_SHOW1);         //雷达显示

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void ClaserDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void ClaserDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
	////////////////////////////////////////////////////////////////////
	 CRect rect;
   //pButtonOne=(CButton*) GetDlgItem(IDC_SHOW1);
   pButtonOne->GetWindowRect(&rect);
   long width=rect.Width();
   long height=rect.Height();

   long px=rect.Width()/2;
   long py=rect.Height()/2;
   //////////////////完成坐标系的绘图
   CClientDC dc(pButtonOne);
  
   CPen pen(PS_DOT,1,RGB(255,0,0));
   CPen *pOldPen=dc.SelectObject(&pen);
   dc.MoveTo(0,rect.Height()/1.5);
   dc.LineTo(rect.Width(),rect.Height()/1.5);
   dc.MoveTo(rect.Width()/2,0);
   dc.LineTo(rect.Width()/2,rect.Height());
   dc.SelectObject(pOldPen);

   
   OnShow();
  
   //OnHangWei();
  
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR ClaserDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}
void ClaserDlg::OnRead() ////////////////////////串口选择处////////////////////////
{
	// TODO: Add your control notification handler code here
	const char* com_port = "COM3";
	const long com_baudrate = 115200;
	// Connect with URG 
	urg_state_t urg_state;
	int ret = urg_connect(&urg_state, com_port, com_baudrate);
	if (ret < 0) 
	{
		getchar();
		exit(1);
	}	
	int max_size = urg_state.max_size;
	long* data = new long [max_size];
	enum { CaptureTimes = 1 };
	//////////////////////////////////////////////////////////////////////
	int recv_n = 0;
	urg_sendMessage("BM", Timeout, &recv_n);
	for (int m = 0; m < CaptureTimes; ++m) 
	{
		urg_captureByGD(&urg_state);
		int n = urg_receiveData(&urg_state, data, max_size);
		if (n > 0) 
		{
		}
	}
	for(int k=0;k<726;k++)
	{
		if (data[k]>5000)
			data[k]=5000;
		/*if (data[k]<20)
			data[k]=20;*/
		if (data[k]==0)
		    data[k]=(2*data[k-1]+data[k+1])/2;
		/*for (int k=725;k>=0;k--)
		{
			if (data[k]>4000)
			data[k]=4000;
		
		    if (data[k]==0)
		    data[k]=(2*data[k+1]+data[k-1])/2;
			date[k]=data[k];
		}*/
		
		date[k]=data[k];
	}
	urg_disconnect();
	delete [] data;
	// If not terminated immidiately. (Remove if not necessary)
    getchar();
	UpdateData(FALSE); //更新编辑框内容

	//m_mid=date[384];
	//m_left=date[45];
	//m_right=date[725];
	m_1=date[45];
	m_2=date[130];
	m_3=date[215];
	m_4=date[300];
	m_5=date[385];
	m_6=date[470];
	m_7=date[555];
	m_8=date[640];
	m_9=date[725];

	UpdateData(FALSE);
	//MessageBox("已读取到数据");
	//return 0;
}

void ClaserDlg::OnShow() 
{
   //////获取当前窗口的坐标系
   /*CRect rect;
   GetDlgItem(IDC_SHOW1)->GetWindowRect(&rect);
   long width=rect.Width();
   long height=rect.Height();
   long px=rect.Width()/2;
   long py=rect.Height()/2;
   //////////////////完成坐标系的绘图
   CClientDC dc(GetDlgItem(IDC_SHOW1));
   CPen pen(PS_DOT,1,RGB(255,0,0));
   CPen *pOldPen=dc.SelectObject(&pen);
   dc.MoveTo(0,rect.Height()/2);
   dc.LineTo(rect.Width(),rect.Height()/2);
   dc.MoveTo(rect.Width()/2,0);
   dc.LineTo(rect.Width()/2,rect.Height());
   dc.SelectObject(pOldPen);*/
   //////画出49个方向的距离值]
   //////预先处理数据（/2）*/
	
   for(int p=44;p<726;p++)
   {
	   date[p]=date[p]/10;
   }
   ///////////////////////
   long fivex[49];
   long fivey[49];
   fivex[0]=date[44]*cos(30*3.14159/180)/2;
   fivey[0]=date[44]*sin(30*3.14159/180)/2;//1
   fivex[1]=date[62]*cos(25*3.14159/180)/2;
   fivey[1]=date[62]*sin(25*3.14159/180)/2;
   fivex[2]=date[76]*cos(20*3.14159/180)/2;
   fivey[2]=date[76]*sin(20*3.14159/180)/2;//2
   fivex[3]=date[90]*cos(15*3.14159/180)/2;
   fivey[3]=date[90]*sin(15*3.14159/180)/2;//3
   fivex[4]=date[104]*cos(10*3.14159/180)/2;
   fivey[4]=date[104]*sin(10*3.14159/180)/2;//4
   fivex[5]=date[118]*cos(5*3.14159/180)/2;
   fivey[5]=date[118]*sin(5*3.14159/180)/2;//5
   fivex[6]=date[132]*cos(0*3.14159/180)/2;
   fivey[6]=date[132]*sin(0*3.14159/180)/2;
   fivex[7]=date[146]*cos(5*3.14159/180)/2;
   fivey[7]=date[146]*sin(5*3.14159/180)/2;
   fivex[8]=date[160]*cos(10*3.14159/180)/2;
   fivey[8]=date[160]*sin(10*3.14159/180)/2;
   fivex[9]=date[174]*cos(15*3.14159/180)/2;
   fivey[9]=date[174]*sin(15*3.14159/180)/2;
   fivex[10]=date[188]*cos(20*3.14159/180)/2;
   fivey[10]=date[188]*sin(20*3.14159/180)/2;
   fivex[11]=date[202]*cos(25*3.14159/180)/2;
   fivey[11]=date[202]*sin(25*3.14159/180)/2;
   fivex[12]=date[216]*cos(30*3.14159/180)/2;
   fivey[12]=date[216]*sin(30*3.14159/180)/2;
   fivex[13]=date[230]*cos(35*3.14159/180)/2;
   fivey[13]=date[230]*sin(35*3.14159/180)/2;
   fivex[14]=date[244]*cos(40*3.14159/180)/2;
   fivey[14]=date[244]*sin(40*3.14159/180)/2;
   fivex[15]=date[258]*cos(45*3.14159/180)/2;
   fivey[15]=date[258]*sin(45*3.14159/180)/2;
   fivex[16]=date[272]*cos(50*3.14159/180)/2;
   fivey[16]=date[272]*sin(50*3.14159/180)/2;
   fivex[17]=date[286]*cos(55*3.14159/180)/2;
   fivey[17]=date[286]*sin(55*3.14159/180)/2;
   fivex[18]=date[300]*cos(60*3.14159/180)/2;
   fivey[18]=date[300]*sin(60*3.14159/180)/2;
   fivex[19]=date[314]*cos(65*3.14159/180)/2;
   fivey[19]=date[314]*sin(65*3.14159/180)/2;
   fivex[20]=date[328]*cos(70*3.14159/180)/2;
   fivey[20]=date[328]*sin(70*3.14159/180)/2;
   fivex[21]=date[343]*cos(75*3.14159/180)/2;
   fivey[21]=date[343]*sin(75*3.14159/180)/2;
   fivex[22]=date[356]*cos(80*3.14159/180)/2;
   fivey[22]=date[356]*sin(80*3.14159/180)/2;
   fivex[23]=date[370]*cos(85*3.14159/180)/2;
   fivey[23]=date[370]*sin(85*3.14159/180)/2;
   fivex[24]=date[384]*cos(90*3.14159/180)/2;
   fivey[24]=date[384]*sin(90*3.14159/180)/2;
   fivex[25]=date[398]*cos(85*3.14159/180)/2;
   fivey[25]=date[398]*sin(85*3.14159/180)/2;
   fivex[26]=date[412]*cos(80*3.14159/180)/2;
   fivey[26]=date[412]*sin(80*3.14159/180)/2;
   fivex[27]=date[426]*cos(75*3.14159/180)/2;
   fivey[27]=date[426]*sin(75*3.14159/180)/2;
   fivex[28]=date[440]*cos(70*3.14159/180)/2;
   fivey[28]=date[440]*sin(70*3.14159/180)/2;
   fivex[29]=date[454]*cos(65*3.14159/180)/2;
   fivey[29]=date[454]*sin(65*3.14159/180)/2;
   fivex[30]=date[468]*cos(60*3.14159/180)/2;
   fivey[30]=date[468]*sin(60*3.14159/180)/2;
   fivex[31]=date[482]*cos(55*3.14159/180)/2;
   fivey[31]=date[482]*sin(55*3.14159/180)/2;
   fivex[32]=date[496]*cos(50*3.14159/180)/2;
   fivey[32]=date[496]*sin(50*3.14159/180)/2;
   fivex[33]=date[510]*cos(45*3.14159/180)/2;
   fivey[33]=date[510]*sin(45*3.14159/180)/2;
   fivex[34]=date[524]*cos(40*3.14159/180)/2;
   fivey[34]=date[524]*sin(40*3.14159/180)/2;
   fivex[35]=date[538]*cos(35*3.14159/180)/2;
   fivey[35]=date[538]*sin(35*3.14159/180)/2;
   fivex[36]=date[552]*cos(30*3.14159/180)/2;
   fivey[36]=date[552]*sin(30*3.14159/180)/2;
   fivex[37]=date[566]*cos(25*3.14159/180)/2;
   fivey[37]=date[566]*sin(25*3.14159/180)/2;
   fivex[38]=date[580]*cos(20*3.14159/180)/2;
   fivey[38]=date[580]*sin(20*3.14159/180)/2;
   fivex[39]=date[594]*cos(15*3.14159/180)/2;
   fivey[39]=date[594]*sin(15*3.14159/180)/2;
   fivex[40]=date[608]*cos(10*3.14159/180)/2;
   fivey[40]=date[608]*sin(10*3.14159/180)/2;
   fivex[41]=date[622]*cos(5*3.14159/180)/2;
   fivey[41]=date[622]*sin(5*3.14159/180)/2;
   fivex[42]=date[636]*cos(0*3.14159/180)/2;
   fivey[42]=date[636]*sin(0*3.14159/180)/2;
   fivex[43]=date[650]*cos(5*3.14159/180)/2;
   fivey[43]=date[650]*sin(5*3.14159/180)/2;
   fivex[44]=date[664]*cos(10*3.14159/180)/2;
   fivey[44]=date[664]*sin(10*3.14159/180)/2;
   fivex[45]=date[678]*cos(15*3.14159/180)/2;
   fivey[45]=date[678]*sin(15*3.14159/180)/2;
   fivex[46]=date[692]*cos(20*3.14159/180)/2;
   fivey[46]=date[692]*sin(20*3.14159/180)/2;
   fivex[47]=date[706]*cos(25*3.14159/180)/2;
   fivey[47]=date[706]*sin(25*3.14159/180)/2;
   fivex[48]=date[725]*cos(30*3.14159/180)/2;
   fivey[48]=date[725]*sin(30*3.14159/180)/2;
   //m_mid=fivex[0];
   CRect rect;
   pButtonOne->GetWindowRect(&rect);
   CClientDC  dc(pButtonOne);
   CPen pen(PS_SOLID,1,RGB(0,0,0));
   CPen *pOldPen=dc.SelectObject(&pen);
   long px=rect.Width()/2;
   long py=rect.Height()/1.5;
   
   
   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[0],py+fivey[0]);
   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[1],py+fivey[1]);
  // dc.MoveTo(px+fivex[0],py+fivey[0]);
   //dc.LineTo(px+fivex[1],py+fivey[1]);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[2],py+fivey[2]);
   //dc.MoveTo(px+fivex[1],py+fivey[1]);
	//dc.LineTo(px+fivex[2],py+fivey[2]);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[3],py+fivey[3]);
 //  dc.MoveTo(px+fivex[2],py+fivey[2]);
	//dc.LineTo(px+fivex[3],py+fivey[3]);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[4],py+fivey[4]);
  // dc.MoveTo(px+fivex[3],py+fivey[3]);
	//dc.LineTo(px+fivex[4],py+fivey[4]);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[5],py+fivey[5]);
  // dc.MoveTo(px+fivex[4],py+fivey[4]);
	//dc.LineTo(px+fivex[5],py+fivey[5]);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[6],py);
 //  dc.MoveTo(px+fivex[5],py+fivey[5]);
	//dc.LineTo(px+fivex[6],py);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[7],py-fivey[7]);
  // dc.MoveTo(px+fivex[6],py);
//	dc.LineTo(px+fivex[7],py-fivey[7]);

   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[8],py-fivey[8]);
  // dc.MoveTo(px+fivex[7],py-fivey[7]);
//	dc.LineTo(px+fivex[8],py-fivey[8]);
  
   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[9],py-fivey[9]);
  //  dc.MoveTo(px+fivex[8],py-fivey[8]);
	//dc.LineTo(px+fivex[9],py-fivey[9]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[10],py-fivey[10]);
 //   dc.MoveTo(px+fivex[9],py-fivey[9]);
	//dc.LineTo(px+fivex[10],py-fivey[10]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[11],py-fivey[11]);
   // dc.MoveTo(px+fivex[10],py-fivey[10]);
	//dc.LineTo(px+fivex[11],py-fivey[11]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[12],py-fivey[12]);
   // dc.MoveTo(px+fivex[11],py-fivey[11]);
	//dc.LineTo(px+fivex[12],py-fivey[12]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[13],py-fivey[13]);
   // dc.MoveTo(px+fivex[12],py-fivey[12]);
	//dc.LineTo(px+fivex[13],py-fivey[13]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[14],py-fivey[14]);
  //  dc.MoveTo(px+fivex[13],py-fivey[13]);
	//dc.LineTo(px+fivex[14],py-fivey[14]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[15],py-fivey[15]);
   // dc.MoveTo(px+fivex[14],py-fivey[14]);
//	dc.LineTo(px+fivex[15],py-fivey[15]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[16],py-fivey[16]);
  //  dc.MoveTo(px+fivex[15],py-fivey[15]);
//	dc.LineTo(px+fivex[16],py-fivey[16]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[17],py-fivey[17]);
 //   dc.MoveTo(px+fivex[16],py-fivey[16]);
//	dc.LineTo(px+fivex[17],py-fivey[17]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[18],py-fivey[18]);
  //  dc.MoveTo(px+fivex[17],py-fivey[17]);
	//dc.LineTo(px+fivex[18],py-fivey[18]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[19],py-fivey[19]);
   // dc.MoveTo(px+fivex[18],py-fivey[18]);
	//dc.LineTo(px+fivex[19],py-fivey[19]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[20],py-fivey[20]);
  //  dc.MoveTo(px+fivex[19],py-fivey[19]);
	//dc.LineTo(px+fivex[20],py-fivey[20]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[21],py-fivey[21]);
   // dc.MoveTo(px+fivex[20],py-fivey[20]);
	//dc.LineTo(px+fivex[21],py-fivey[21]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[22],py-fivey[22]);
   // dc.MoveTo(px+fivex[21],py-fivey[21]);
	//dc.LineTo(px+fivex[22],py-fivey[22]);


   dc.MoveTo(px,py);
   dc.LineTo(px+fivex[23],py-fivey[23]);
   // dc.MoveTo(px+fivex[22],py-fivey[22]);
	//dc.LineTo(px+fivex[23],py-fivey[23]);


   dc.MoveTo(px,py);
   dc.LineTo(px,py-fivey[24]);
   // dc.MoveTo(px+fivex[23],py-fivey[23]);
	//dc.LineTo(px,py-fivey[24]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[25],py-fivey[25]);
   // dc.MoveTo(px,py-fivey[24]);
//	dc.LineTo(px-fivex[25],py-fivey[25]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[26],py-fivey[26]);
  //  dc.MoveTo(px-fivex[25],py-fivey[25]);
//	dc.LineTo(px-fivex[26],py-fivey[26]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[27],py-fivey[27]);
 //   dc.MoveTo(px-fivex[26],py-fivey[26]);
//	dc.LineTo(px-fivex[27],py-fivey[27]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[28],py-fivey[28]);
   // dc.MoveTo(px-fivex[27],py-fivey[27]);
//	dc.LineTo(px-fivex[28],py-fivey[28]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[29],py-fivey[29]);
   // dc.MoveTo(px-fivex[28],py-fivey[28]);
//	dc.LineTo(px-fivex[29],py-fivey[29]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[30],py-fivey[30]);
 //   dc.MoveTo(px-fivex[29],py-fivey[29]);
//	dc.LineTo(px-fivex[30],py-fivey[30]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[31],py-fivey[31]);
  //  dc.MoveTo(px-fivex[30],py-fivey[30]);
//	dc.LineTo(px-fivex[31],py-fivey[31]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[32],py-fivey[32]);
   // dc.MoveTo(px-fivex[31],py-fivey[31]);
//	dc.LineTo(px-fivex[32],py-fivey[32]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[33],py-fivey[33]);
  //  dc.MoveTo(px-fivex[32],py-fivey[32]);
	//dc.LineTo(px-fivex[33],py-fivey[33]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[34],py-fivey[34]);
  //  dc.MoveTo(px-fivex[33],py-fivey[33]);
//	dc.LineTo(px-fivex[34],py-fivey[34]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[35],py-fivey[35]);
 //   dc.MoveTo(px-fivex[34],py-fivey[34]);
	//dc.LineTo(px-fivex[35],py-fivey[35]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[36],py-fivey[36]);
 //   dc.MoveTo(px-fivex[35],py-fivey[35]);
//	dc.LineTo(px-fivex[36],py-fivey[36]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[37],py-fivey[37]);
   // dc.MoveTo(px-fivex[36],py-fivey[36]);
//	dc.LineTo(px-fivex[37],py-fivey[37]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[38],py-fivey[38]);
 //   dc.MoveTo(px-fivex[37],py-fivey[37]);
//	dc.LineTo(px-fivex[38],py-fivey[38]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[39],py-fivey[39]);
  //  dc.MoveTo(px-fivex[38],py-fivey[38]);
	//dc.LineTo(px-fivex[39],py-fivey[39]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[40],py-fivey[40]);
   // dc.MoveTo(px-fivex[39],py-fivey[39]);
//	dc.LineTo(px-fivex[40],py-fivey[40]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[41],py-fivey[41]);
  //  dc.MoveTo(px-fivex[40],py-fivey[40]);
	//dc.LineTo(px-fivex[41],py-fivey[41]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[42],py);
 //   dc.MoveTo(px-fivex[41],py-fivey[41]);
//	dc.LineTo(px-fivex[42],py);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[43],py+fivey[43]);
 //   dc.MoveTo(px-fivex[42],py);
//	dc.LineTo(px-fivex[43],py+fivey[43]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[44],py+fivey[44]);
 //   dc.MoveTo(px-fivex[43],py+fivey[43]);
//	dc.LineTo(px-fivex[44],py+fivey[44]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[45],py+fivey[45]);
  //  dc.MoveTo(px-fivex[44],py+fivey[44]);
	//dc.LineTo(px-fivex[45],py+fivey[45]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[46],py+fivey[46]);
   // dc.MoveTo(px-fivex[45],py+fivey[45]);
	//dc.LineTo(px-fivex[46],py+fivey[46]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[47],py+fivey[47]);
  // dc.MoveTo(px-fivex[46],py+fivey[46]);
	//dc.LineTo(px-fivex[47],py+fivey[47]);


   dc.MoveTo(px,py);
   dc.LineTo(px-fivex[48],py+fivey[48]);
  //  dc.MoveTo(px-fivex[47],py+fivey[47]);
	//dc.LineTo(px-fivex[48],py+fivey[48]);



  /* dc.MoveTo(px,py);
   //prex[2]=px;
   //prey[2]=py-date[384];
   dc.LineTo(px,py-date[384]/2);///////3

   dc.MoveTo(px,py);
   //prex[3]=px+fivex[3];
   //prey[3]=py-fivey[3];
   dc.LineTo(px-fivex[3],py-fivey[3]);////4

   dc.MoveTo(px,py);
   //prex[4]=px+fivex[4];
   //prey[4]=py+fivey[4];
   dc.LineTo(px-fivex[4],py+fivey[4]);////5*/
   
   dc.SelectObject(pOldPen);

}
void ClaserDlg::OnStartScan() 
{
	// TODO: Add your control notification handler code here

	SetTimer(1,1000,NULL);
	
	
}

void ClaserDlg::OnTimer(UINT nIDEvent) 
{
	// TODO: Add your message handler code here and/or call default
	
    
	OnRead();
    
	Invalidate(true);
    
	
	CDialog::OnTimer(nIDEvent);
}

void ClaserDlg::OnStopScan() 
{
	// TODO: Add your control notification handler code here
	KillTimer(1);
	
}
//////////////////////////////////////////////////////////////////////

