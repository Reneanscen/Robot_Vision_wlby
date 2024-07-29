//////////////////////////////////////////////////////////////////////
// Matrix.h
//////////////////////////////////////////////////////////////////////
#ifndef _TANGTANG_CMATRIX_
#define _TANGTANG_CMATRIX_

//#include <windows.h>
#include <iostream>
#include <cmath>
//#include "WIN32_DLL_IMPORT_4_H.h"

//class __DLL_CLASS CMatrix  
class  CMatrix 
{
	//
	// ���нӿں���
	//
public:

	//
	// ����������
	//

	CMatrix();										// �������캯��
	CMatrix(int nRows, int nCols);					// ָ�����й��캯��
	CMatrix(int nRows, int nCols,const double value[]);	// ָ�����ݹ��캯��
	CMatrix(int nSize);								// �����캯��
	CMatrix(int nSize, const double value[]);				// ָ�����ݷ����캯��
	CMatrix(const CMatrix& other);					// �������캯��
	bool	Init(int nRows, int nCols);				// ��ʼ������	
	bool	MakeUnitMatrix(int nSize);				// �������ʼ��Ϊ��λ����
	bool MakeZeroMatrix(int row, int clo);
	bool ResetToZeroMatrix();

	virtual ~CMatrix();								// ��������

	////
	//// ��������ʾ
	////
	// �����ʾ
	bool print(void);
	void printByNoSmall(double small_margin = 0.0000001);


	//// ���ַ���ת��Ϊ��������
	//bool FromString(CString s, const CString& sDelim = " ", bool bLineBreak = TRUE);	
	//// ������ת��Ϊ�ַ���
	//CString ToString(const CString& sDelim = " ", bool bLineBreak = TRUE) const;
	//// �������ָ����ת��Ϊ�ַ���
	//CString RowToString(int nRow, const CString& sDelim = " ") const;
	//// �������ָ����ת��Ϊ�ַ���
	//CString ColToString(int nCol, const CString& sDelim = " ") const;

	//
	// Ԫ����ֵ����
	//

	bool	SetElement(int nRow, int nCol,double value);	// ����ָ��Ԫ�ص�ֵ
	double	GetElement(int nRow, int nCol) const;			// ��ȡָ��Ԫ�ص�ֵ
	void    SetData(const double value[]);						// ���þ����ֵ
	void	SetDataF(double value[], int nRows, int nCols);	// ���þ����ֵ������Fortranר�á����ã���
	void    SetDataVertical(double value[]);						// ���þ����ֵ��ֱ����
	int		GetNumColumns() const;							// ��ȡ���������
	int		GetNumRows() const;								// ��ȡ���������
	int     GetRowVector(int nRow, double* pVector) const;	// ��ȡ�����ָ���о���
	int     GetColVector(int nCol, double* pVector) const;	// ��ȡ�����ָ���о���
	double* GetData() const;								// ��ȡ�����ֵ

	//
	// ��ѧ����
	//

	const CMatrix& operator=(const CMatrix& other);
	bool operator==(const CMatrix& other) const;
	bool operator!=(const CMatrix& other) const;
	CMatrix	operator|(const CMatrix& other) const;		// Column Add
	CMatrix	operator&(const CMatrix& other) const;		// Row Add
	CMatrix	operator+(const CMatrix& other) const;
	CMatrix	operator-(const CMatrix& other) const;
	CMatrix	operator*(double value) const;
	CMatrix	operator*(const CMatrix& other) const;

	double& operator()(int row, int clo);

	// ������˷�
	bool CMul(const CMatrix& AR, const CMatrix& AI, const CMatrix& BR, const CMatrix& BI, CMatrix& CR, CMatrix& CI) const;
	// �����ת��
	CMatrix Transpose() const;

	double operator[](int index) const;

	//
	// �㷨
	//

	// ʵ���������ȫѡ��Ԫ��˹��Լ����
	bool InvertGaussJordan();                                               
	// �����������ȫѡ��Ԫ��˹��Լ����
	bool InvertGaussJordan(CMatrix& mtxImag);                                 
	// �Գ��������������
	bool InvertSsgj();                                              
	// �в����Ⱦ�������İ����ط���
	bool InvertTrench();                                                    
	// ������ʽֵ��ȫѡ��Ԫ��˹��ȥ��
	double DetGauss();                                                              
	// ������ȵ�ȫѡ��Ԫ��˹��ȥ��
	int RankGauss();
	// �Գ��������������˹���ֽ�������ʽ����ֵ
	bool DetCholesky(double* dblDet);                                                               
	// ��������Ƿֽ�
	bool SplitLU(CMatrix& mtxL, CMatrix& mtxU);                                     
	// һ��ʵ�����QR�ֽ�
	bool SplitQR(CMatrix& mtxQ);                                                      
	// һ��ʵ���������ֵ�ֽ�
	bool SplitUV(CMatrix& mtxU, CMatrix& mtxV, double eps = 0.000001);                                       
	// ������������ֵ�ֽⷨ
	bool GInvertUV(CMatrix& mtxAP, CMatrix& mtxU, CMatrix& mtxV, double eps = 0.000001);
	// Լ���Գƾ���Ϊ�Գ����Խ���ĺ�˹�ɶ��±任��
	bool MakeSymTri(CMatrix& mtxQ, CMatrix& mtxT, double dblB[], double dblC[]);
	// ʵ�Գ����Խ����ȫ������ֵ�����������ļ���
	bool SymTriEigenv(double dblB[], double dblC[], CMatrix& mtxQ, int nMaxIt = 60, double eps = 0.000001);
	// Լ��һ��ʵ����Ϊ���겮�����ĳ������Ʊ任��
	void MakeHberg();
	// ����겮�����ȫ������ֵ��QR����
	bool HBergEigenv(double dblU[], double dblV[], int nMaxIt = 60, double eps = 0.000001);
	// ��ʵ�Գƾ�������ֵ�������������ſɱȷ�
	bool JacobiEigenv(double dblEigenValue[], CMatrix& mtxEigenVector, int nMaxIt = 60, double eps = 0.000001);
	// ��ʵ�Գƾ�������ֵ�������������ſɱȹ��ط�
	bool JacobiEigenv2(double dblEigenValue[], CMatrix& mtxEigenVector, double eps = 0.000001);



	// Get the sub matrix from the parents' matrix
	CMatrix GetSubMatrix(int row1, int row2, int col1, int col2);

	double Norm(void);

	static double EuclideanDistance(CMatrix mat1, CMatrix mat2);


	CMatrix GetColVectorMat(int nCol);


	//
	// ���������ݳ�Ա
	//
protected:
	int	m_nNumColumns;			// ��������
	int	m_nNumRows;				// ��������
	double*	m_pData;			// �������ݻ�����

	//
	// �ڲ�����
	//
private:
	void ppp(double a[], double e[], double s[], double v[], int m, int n);
	void sss(double fg[2], double cs[2]);


//added by zihan---2011/10/12
public:
	CMatrix(int nSize,char u);//u=='u'ʱ���쵥λ��
	CMatrix(int nRows, int nCols,char o,double val);//o='o'ʱ����Ԫ��ȫΪval�ľ���
	bool unit();//��λ������
	bool block_equal(const CMatrix& other,int row1, int row2, int col1, int col2);//��other��ȥ��ֵ����Ŀ飬Ҳ�����������ž���
    bool CMsqrt(CMatrix& result);//ʵ�Գư��������󿪷�

public:
	static void plus(const CMatrix &a, const CMatrix &b, CMatrix &result);
	static void minus(const CMatrix &a, const CMatrix &b, CMatrix &result);
	static void multiple(const CMatrix &a, double value, CMatrix &result);
	static void multiple(const CMatrix &a, const CMatrix &b, CMatrix &result);
	static void Transpose(const CMatrix &a, CMatrix &result);

};
	CMatrix eye(int nSize);//�õ�nSizeά�ĵ�λ����

#endif
