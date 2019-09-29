
// laserDlg.h : 头文件
//

#pragma once


// ClaserDlg 对话框
class ClaserDlg : public CDialogEx
{
// 构造
public:
	ClaserDlg(CWnd* pParent = NULL);	// 标准构造函数
	//void OnHangWei();
	//CButton* pButtonTwo;           //path
	//void OnSHOW();
	CButton* pButtonOne;

// 对话框数据
	enum { IDD = IDD_LASER_DIALOG };
	long	m_rece;
//	long	m_mid;
//	long	m_right;
//	long	m_left;
	long	m_width;
	long	m_height;
	long	m_gox;  /////目标位置
	long	m_goy;
	long	m_inx;  //////初始位置
	long	m_iny;
	long	m_tx;
	long	m_ty;
	double	m_corner;
	long	m_distance;
	long    px;
	long    py;
	long    dSn;
	long    sitan;
	float	m_HMRHangxj;
	float	m_HMRHenggj;
	float	m_HMRFuyj;

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	////////////////////////////////////////////////////////
	afx_msg void OnRead();
	afx_msg void OnShow();
	afx_msg void OnPlan();
	afx_msg void OnStartScan();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnStopScan();
	DECLARE_MESSAGE_MAP()
	private:
	CPtrArray m_ptrArray;
public:
//	CEdit m_1;
	long m_1;
	long m_2;
	long m_3;
	long m_4;
	long m_5;
	long m_6;
	long m_7;
	long m_8;
	long m_9;
};

