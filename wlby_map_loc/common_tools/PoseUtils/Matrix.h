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
	// 公有接口函数
	//
public:

	//
	// 构造与析构
	//

	CMatrix();										// 基础构造函数
	CMatrix(int nRows, int nCols);					// 指定行列构造函数
	CMatrix(int nRows, int nCols,const double value[]);	// 指定数据构造函数
	CMatrix(int nSize);								// 方阵构造函数
	CMatrix(int nSize, const double value[]);				// 指定数据方阵构造函数
	CMatrix(const CMatrix& other);					// 拷贝构造函数
	bool	Init(int nRows, int nCols);				// 初始化矩阵	
	bool	MakeUnitMatrix(int nSize);				// 将方阵初始化为单位矩阵
	bool MakeZeroMatrix(int row, int clo);
	bool ResetToZeroMatrix();

	virtual ~CMatrix();								// 析构函数

	////
	//// 输入与显示
	////
	// 输出显示
	bool print(void);
	void printByNoSmall(double small_margin = 0.0000001);


	//// 将字符串转换为矩阵数据
	//bool FromString(CString s, const CString& sDelim = " ", bool bLineBreak = TRUE);	
	//// 将矩阵转换为字符串
	//CString ToString(const CString& sDelim = " ", bool bLineBreak = TRUE) const;
	//// 将矩阵的指定行转换为字符串
	//CString RowToString(int nRow, const CString& sDelim = " ") const;
	//// 将矩阵的指定列转换为字符串
	//CString ColToString(int nCol, const CString& sDelim = " ") const;

	//
	// 元素与值操作
	//

	bool	SetElement(int nRow, int nCol,double value);	// 设置指定元素的值
	double	GetElement(int nRow, int nCol) const;			// 获取指定元素的值
	void    SetData(const double value[]);						// 设置矩阵的值
	void	SetDataF(double value[], int nRows, int nCols);	// 设置矩阵的值（调用Fortran专用。慎用！）
	void    SetDataVertical(double value[]);						// 设置矩阵的值竖直方向
	int		GetNumColumns() const;							// 获取矩阵的列数
	int		GetNumRows() const;								// 获取矩阵的行数
	int     GetRowVector(int nRow, double* pVector) const;	// 获取矩阵的指定行矩阵
	int     GetColVector(int nCol, double* pVector) const;	// 获取矩阵的指定列矩阵
	double* GetData() const;								// 获取矩阵的值

	//
	// 数学操作
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

	// 复矩阵乘法
	bool CMul(const CMatrix& AR, const CMatrix& AI, const CMatrix& BR, const CMatrix& BI, CMatrix& CR, CMatrix& CI) const;
	// 矩阵的转置
	CMatrix Transpose() const;

	double operator[](int index) const;

	//
	// 算法
	//

	// 实矩阵求逆的全选主元高斯－约当法
	bool InvertGaussJordan();                                               
	// 复矩阵求逆的全选主元高斯－约当法
	bool InvertGaussJordan(CMatrix& mtxImag);                                 
	// 对称正定矩阵的求逆
	bool InvertSsgj();                                              
	// 托伯利兹矩阵求逆的埃兰特方法
	bool InvertTrench();                                                    
	// 求行列式值的全选主元高斯消去法
	double DetGauss();                                                              
	// 求矩阵秩的全选主元高斯消去法
	int RankGauss();
	// 对称正定矩阵的乔里斯基分解与行列式的求值
	bool DetCholesky(double* dblDet);                                                               
	// 矩阵的三角分解
	bool SplitLU(CMatrix& mtxL, CMatrix& mtxU);                                     
	// 一般实矩阵的QR分解
	bool SplitQR(CMatrix& mtxQ);                                                      
	// 一般实矩阵的奇异值分解
	bool SplitUV(CMatrix& mtxU, CMatrix& mtxV, double eps = 0.000001);                                       
	// 求广义逆的奇异值分解法
	bool GInvertUV(CMatrix& mtxAP, CMatrix& mtxU, CMatrix& mtxV, double eps = 0.000001);
	// 约化对称矩阵为对称三对角阵的豪斯荷尔德变换法
	bool MakeSymTri(CMatrix& mtxQ, CMatrix& mtxT, double dblB[], double dblC[]);
	// 实对称三对角阵的全部特征值与特征向量的计算
	bool SymTriEigenv(double dblB[], double dblC[], CMatrix& mtxQ, int nMaxIt = 60, double eps = 0.000001);
	// 约化一般实矩阵为赫申伯格矩阵的初等相似变换法
	void MakeHberg();
	// 求赫申伯格矩阵全部特征值的QR方法
	bool HBergEigenv(double dblU[], double dblV[], int nMaxIt = 60, double eps = 0.000001);
	// 求实对称矩阵特征值与特征向量的雅可比法
	bool JacobiEigenv(double dblEigenValue[], CMatrix& mtxEigenVector, int nMaxIt = 60, double eps = 0.000001);
	// 求实对称矩阵特征值与特征向量的雅可比过关法
	bool JacobiEigenv2(double dblEigenValue[], CMatrix& mtxEigenVector, double eps = 0.000001);



	// Get the sub matrix from the parents' matrix
	CMatrix GetSubMatrix(int row1, int row2, int col1, int col2);

	double Norm(void);

	static double EuclideanDistance(CMatrix mat1, CMatrix mat2);


	CMatrix GetColVectorMat(int nCol);


	//
	// 保护性数据成员
	//
protected:
	int	m_nNumColumns;			// 矩阵列数
	int	m_nNumRows;				// 矩阵行数
	double*	m_pData;			// 矩阵数据缓冲区

	//
	// 内部函数
	//
private:
	void ppp(double a[], double e[], double s[], double v[], int m, int n);
	void sss(double fg[2], double cs[2]);


//added by zihan---2011/10/12
public:
	CMatrix(int nSize,char u);//u=='u'时构造单位阵
	CMatrix(int nRows, int nCols,char o,double val);//o='o'时构造元素全为val的矩阵
	bool unit();//单位化方阵
	bool block_equal(const CMatrix& other,int row1, int row2, int col1, int col2);//用other阵去赋值矩阵的块，也可以用于扩张矩阵
    bool CMsqrt(CMatrix& result);//实对称半正定矩阵开方

public:
	static void plus(const CMatrix &a, const CMatrix &b, CMatrix &result);
	static void minus(const CMatrix &a, const CMatrix &b, CMatrix &result);
	static void multiple(const CMatrix &a, double value, CMatrix &result);
	static void multiple(const CMatrix &a, const CMatrix &b, CMatrix &result);
	static void Transpose(const CMatrix &a, CMatrix &result);

};
	CMatrix eye(int nSize);//得到nSize维的单位矩阵

#endif
