#pragma warning(disable:4996)

#if defined(_WIN32_WINNT) && _WIN32_WINNT<0x0500
#undef _WIN32_WINNT
#endif

#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x500
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <windows.h>
#include <direct.h>
#include <gl/gl.h>
#include <gl/glu.h>

#include "fssimplewindow.h"

#pragma comment(lib,"kernel32.lib")
#pragma comment(lib,"user32.lib")
#pragma comment(lib,"gdi32.lib")
#pragma comment(lib,"opengl32.lib")
#pragma comment(lib,"glu32.lib")



const unsigned int FS_NUM_VK=256;


class FsWin32KeyMapper
{
public:
	FsWin32KeyMapper();
	~FsWin32KeyMapper();

	int VkToFsKey(int vk);
	int FsKeyToVk(int fsKey);

protected:
	void AddKeyMapping(int fskey,int vk);

	int *mapVKtoFSKEY;
	int *mapFSKEYtoVK;
};

static FsWin32KeyMapper fsKeyMapper;



class FsSimpleWindowInternal
{
public:
	HWND hWnd;
	HGLRC hRC;
	HPALETTE hPlt;
	HDC hDC;

	FsSimpleWindowInternal();
};



static FsSimpleWindowInternal fsWin32Internal;

FsSimpleWindowInternal::FsSimpleWindowInternal()
{
	hWnd=NULL;
	hRC=NULL;
	hPlt=NULL;
	hDC=NULL;
}

const FsSimpleWindowInternal *FsGetSimpleWindowInternal(void)
{
	return &fsWin32Internal;
}


static LONG WINAPI WindowFunc(HWND wnd,UINT msg,WPARAM wp,LPARAM lp);
static void YsSetPixelFormat(HDC dc);
static HPALETTE YsCreatePalette(HDC dc);
static void InitializeOpenGL(HWND wnd);



// For OpenGL set up >>
static int doubleBuffer=0;
// For OpenGL set up <<


class FsMouseEventLog
{
public:
	int eventType;
	int lb,mb,rb;
	int mx,my;
	unsigned int shift,ctrl;
};


#define NKEYBUF 256
static int keyBuffer[NKEYBUF];
static int nKeyBufUsed=0;
static int charBuffer[NKEYBUF];
static int nCharBufUsed=0;
static int nMosBufUsed=0;
static FsMouseEventLog mosBuffer[NKEYBUF];

static int exposure=0;


#define WINSTYLE WS_OVERLAPPED|WS_CAPTION|WS_VISIBLE|WS_SYSMENU|WS_THICKFRAME|WS_MINIMIZEBOX|WS_MAXIMIZEBOX

static const char *WINCLASS="FsSimpleWindow_MainWindowClass";
static const char *WINNAME="MainWindow";

void FsOpenWindow(int x0,int y0,int wid,int hei,int useDoubleBuffer)
{
	if(NULL!=fsWin32Internal.hWnd)
	{
		MessageBoxA(fsWin32Internal.hWnd,"Error! Window already exists.","Error!",MB_OK);
		exit(1);
	}

	// Note 2012/03/08 RegisterClassW and CreateWindowW doesn't seem to work.
	WNDCLASSA wc;
	HINSTANCE inst=GetModuleHandleA(NULL);

	wc.style=CS_OWNDC|CS_BYTEALIGNWINDOW;
	wc.lpfnWndProc=(WNDPROC)WindowFunc;
	wc.cbClsExtra=0;
	wc.cbWndExtra=0;
	wc.hInstance=(HINSTANCE)inst;
	wc.hIcon=LoadIconA(inst,"MAINICON");
	wc.hCursor=LoadCursor(NULL,IDC_ARROW);
	wc.hbrBackground=(HBRUSH)GetStockObject(WHITE_BRUSH);
	wc.lpszMenuName=NULL;
	wc.lpszClassName=WINCLASS;
	if(0!=RegisterClassA(&wc))
	{
		doubleBuffer=useDoubleBuffer;


		RECT rc;
		rc.left  =x0;
		rc.top   =y0;
		rc.right =(unsigned long)(x0+wid-1);
		rc.bottom=(unsigned long)(y0+hei-1);
		AdjustWindowRect(&rc,WINSTYLE,FALSE);
		wid  =rc.right-rc.left+1;
		hei  =rc.bottom-rc.top+1;

#ifdef _UNICODE
		// What's the point of using CreateWindowA?  Another weird Microsoft logic here.
		static wchar_t buf[256];
		const char *windowNameA=(const char *)WINNAME;
		for(int i=0; i<255 && 0!=windowNameA[i]; ++i)
		{
			buf[i]=windowNameA[i];
			buf[i+1]=0;
		}
		const char *windowNameUsed=(const char *)buf;
#else
		const char *windowNameUsed=(const char *)WINNAME;
#endif

		fsWin32Internal.hWnd=CreateWindowA(WINCLASS,windowNameUsed,WINSTYLE,x0,y0,wid,hei,NULL,NULL,inst,NULL);
		if(NULL!=fsWin32Internal.hWnd)
		{
			InitializeOpenGL(fsWin32Internal.hWnd);

			ShowWindow(fsWin32Internal.hWnd,SW_SHOWNORMAL);
			UpdateWindow(fsWin32Internal.hWnd);

			FsPassedTime();  // Reset Timer
		}
		else
		{
			printf("Could not open window.\n");
			exit(1);
		}
	}
}

void FsCloseWindow(void)
{
	DestroyWindow(fsWin32Internal.hWnd);
	fsWin32Internal.hWnd=NULL;
	fsWin32Internal.hRC=NULL;
	fsWin32Internal.hPlt=NULL;
	fsWin32Internal.hDC=NULL;
}

void FsGetWindowSize(int &wid,int &hei)
{
	RECT rect;
	GetClientRect(fsWin32Internal.hWnd,&rect);
	wid=rect.right;
	hei=rect.bottom;
}

void FsPollDevice(void)
{
	MSG msg;
	while(PeekMessage(&msg,fsWin32Internal.hWnd,0,0,PM_REMOVE))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
}

void FsSleep(int ms)
{
	if(ms>0)
	{
		Sleep(ms);
	}
}

long long int FsPassedTime(void)
{
	static unsigned int lastTick;
	unsigned int tick,passed;

	tick=GetTickCount();
	if(lastTick<tick)
	{
		passed=tick-lastTick;
	}
	else
	{
		passed=1;
	}
	lastTick=tick;

	return passed;
}

int FsInkey(void)
{
	if(nKeyBufUsed>0)
	{
		int i,keyCode;
		keyCode=keyBuffer[0];
		nKeyBufUsed--;
		for(i=0; i<nKeyBufUsed; i++)
		{
			keyBuffer[i]=keyBuffer[i+1];
		}
		return keyCode;
	}
	return 0;
}

int FsInkeyChar(void)
{
	if(nCharBufUsed>0)
	{
		int i,asciiCode;
		asciiCode=charBuffer[0];
		nCharBufUsed--;
		for(i=0; i<nCharBufUsed; i++)
		{
			charBuffer[i]=charBuffer[i+1];
		}
		return asciiCode;
	}
	return 0;
}

int FsGetKeyState(int fsKeyCode)
{
	const int vk=fsKeyMapper.FsKeyToVk(fsKeyCode);
	return (GetKeyState(vk)&0x8000)!=0;
}

int FsCheckWindowExposure(void)
{
	const int ret=exposure;
	exposure=0;
	return ret;
}

void FsGetMouseState(int &lb,int &mb,int &rb,int &mx,int &my)
{
	POINT cur;
	GetCursorPos(&cur);
	ScreenToClient(fsWin32Internal.hWnd,&cur);

	mx=cur.x;
	my=cur.y;

	lb=((GetKeyState(VK_LBUTTON)&0x8000)!=0 ? 1 : 0);
	mb=((GetKeyState(VK_MBUTTON)&0x8000)!=0 ? 1 : 0);
	rb=((GetKeyState(VK_RBUTTON)&0x8000)!=0 ? 1 : 0);
}

int FsGetMouseEvent(int &lb,int &mb,int &rb,int &mx,int &my)
{
	if(0<nMosBufUsed)
	{
		int eventType=mosBuffer[0].eventType;
		mx=mosBuffer[0].mx;
		my=mosBuffer[0].my;
		lb=mosBuffer[0].lb;
		mb=mosBuffer[0].mb;
		rb=mosBuffer[0].rb;

		int i;
		for(i=0; i<nMosBufUsed-1; i++)
		{
			mosBuffer[i]=mosBuffer[i+1];
		}
		nMosBufUsed--;

		return eventType;
	}
	else
	{
		FsGetMouseState(lb,mb,rb,mx,my);
		return FSMOUSEEVENT_NONE;
	}
}

void FsSwapBuffers(void)
{
	if(0==doubleBuffer)
	{
		MessageBoxA(NULL,"Error! FsSwapBuffers used in the single-buffered mode.","Error!",MB_OK);
		exit(1);
	}

	HDC hDC;
	glFlush();
	hDC=wglGetCurrentDC();
	SwapBuffers(hDC);
}

static LONG WINAPI WindowFunc(HWND hWnd,UINT msg,WPARAM wp,LPARAM lp)
{
	switch(msg)
	{
	case WM_QUERYNEWPALETTE:
	case WM_PALETTECHANGED:
		if(NULL!=fsWin32Internal.hPlt)
		{
			SelectPalette(fsWin32Internal.hDC,fsWin32Internal.hPlt,FALSE);
			RealizePalette(fsWin32Internal.hDC);
		}
		return DefWindowProc(hWnd,msg,wp,lp);
	case WM_CREATE:
		fsWin32Internal.hDC=GetDC(hWnd);
		YsSetPixelFormat(fsWin32Internal.hDC);
		fsWin32Internal.hRC=wglCreateContext(fsWin32Internal.hDC);
		wglMakeCurrent(fsWin32Internal.hDC,fsWin32Internal.hRC);
		if(0==doubleBuffer)
		{
			glDrawBuffer(GL_FRONT);
		}
		InitializeOpenGL(hWnd);
		break;
	case WM_SIZE:
		wglMakeCurrent(fsWin32Internal.hDC,fsWin32Internal.hRC);
		break;
	case WM_PAINT:
		wglMakeCurrent(fsWin32Internal.hDC,fsWin32Internal.hRC);
		exposure=1;
		return DefWindowProc(hWnd,msg,wp,lp);
	case WM_COMMAND:
		break;
	case WM_DESTROY:
		exit(1);
		break;
	case WM_MOUSEWHEEL:
		{
			int step;
			step=HIWORD(wp);
			if(step>=0x8000)
			{
				step-=0x10000;
			}
			step/=WHEEL_DELTA;
			if(step>0)
			{
				while(step>0)
				{
					if(nKeyBufUsed<NKEYBUF)
					{
						keyBuffer[nKeyBufUsed++]=FSKEY_WHEELUP;
					}
					step--;
				}
			}
			else if(step<0)
			{
				while(step<0)
				{
					if(nKeyBufUsed<NKEYBUF)
					{
						keyBuffer[nKeyBufUsed++]=FSKEY_WHEELDOWN;
					}
					step++;
				}
			}
		}
		break;
	case WM_SYSKEYDOWN:
		if((lp & (1<<29))!=0 && // Alt
		  (wp==VK_MENU ||
		   wp==VK_OEM_1 ||
		   wp==VK_OEM_PLUS ||
		   wp==VK_OEM_COMMA ||
		   wp==VK_OEM_MINUS ||
		   wp==VK_OEM_PERIOD ||
		   wp==VK_OEM_2 ||
		   wp==VK_OEM_3 ||
		   wp==VK_OEM_4 ||
		   wp==VK_OEM_5 ||
		   wp==VK_OEM_6 ||
		   wp==VK_OEM_7 ||
		   wp==VK_OEM_8 ||
#ifdef VK_OEM_AX
		   wp==VK_OEM_AX ||
#endif
		   wp==VK_OEM_102 ||
		   wp=='0' ||
		   wp=='1' ||
		   wp=='2' ||
		   wp=='3' ||
		   wp=='4' ||
		   wp=='5' ||
		   wp=='6' ||
		   wp=='7' ||
		   wp=='8' ||
		   wp=='9' ||
		   wp=='A' ||
		   wp=='B' ||
		   wp=='C' ||
		   wp=='D' ||
		   wp=='E' ||
		   wp=='F' ||
		   wp=='G' ||
		   wp=='H' ||
		   wp=='I' ||
		   wp=='J' ||
		   wp=='K' ||
		   wp=='L' ||
		   wp=='M' ||
		   wp=='N' ||
		   wp=='O' ||
		   wp=='P' ||
		   wp=='Q' ||
		   wp=='R' ||
		   wp=='S' ||
		   wp=='T' ||
		   wp=='U' ||
		   wp=='V' ||
		   wp=='W' ||
		   wp=='X' ||
		   wp=='Y' ||
		   wp=='Z' ||
		   wp==VK_ESCAPE ||
		   wp==VK_F1 ||
		   wp==VK_F2 ||
		   wp==VK_F3 ||
		   /* wp==VK_F4 || */
		   wp==VK_F5 ||
		   wp==VK_F6 ||
		   wp==VK_F7 ||
		   wp==VK_F8 ||
		   wp==VK_F9 ||
		   wp==VK_F10 ||
		   wp==VK_F11 ||
		   wp==VK_F12 ||
		   wp==VK_RETURN ||
		   wp==VK_NUMLOCK ||
		   wp==VK_NUMPAD0 ||
		   wp==VK_NUMPAD1 ||
		   wp==VK_NUMPAD2 ||
		   wp==VK_NUMPAD3 ||
		   wp==VK_NUMPAD4 ||
		   wp==VK_NUMPAD5 ||
		   wp==VK_NUMPAD6 ||
		   wp==VK_NUMPAD7 ||
		   wp==VK_NUMPAD8 ||
		   wp==VK_NUMPAD9 ||
		   wp==VK_DECIMAL ||
		   wp==VK_DIVIDE ||
		   wp==VK_MULTIPLY ||
		   wp==VK_SUBTRACT ||
		   wp==VK_ADD))
		{
			int keyCode;
			keyCode=fsKeyMapper.VkToFsKey(wp);
			if(keyCode!=0 && nKeyBufUsed<NKEYBUF)
			{
				keyBuffer[nKeyBufUsed++]=keyCode;
			}
			return 0;
		}
		return DefWindowProc(hWnd,msg,wp,lp);
	case WM_SYSKEYUP:
		return 0;
	case WM_KEYDOWN:
		if(nKeyBufUsed<NKEYBUF)
		{
			int keyCode;
			keyCode=fsKeyMapper.VkToFsKey(wp);
			if(keyCode!=0)
			{
				keyBuffer[nKeyBufUsed++]=keyCode;
			}
		}
		break;
	case WM_CHAR:
		if(nCharBufUsed<NKEYBUF)
		{
			charBuffer[nCharBufUsed++]=wp;
		}
		break;
	case WM_ERASEBKGND:
		return 1;

	case WM_LBUTTONDOWN:
	case WM_LBUTTONUP:
	case WM_MBUTTONDOWN:
	case WM_MBUTTONUP:
	case WM_RBUTTONDOWN:
	case WM_RBUTTONUP:
	case WM_MOUSEMOVE:
		if(nMosBufUsed<NKEYBUF)
		{
			int eventType;
			switch(msg)
			{
			default:
				eventType=FSMOUSEEVENT_NONE;
				break;
			case WM_LBUTTONDOWN:
				eventType=FSMOUSEEVENT_LBUTTONDOWN;
				break;
			case WM_LBUTTONUP:
				eventType=FSMOUSEEVENT_LBUTTONUP;
				break;
			case WM_MBUTTONDOWN:
				eventType=FSMOUSEEVENT_MBUTTONDOWN;
				break;
			case WM_MBUTTONUP:
				eventType=FSMOUSEEVENT_MBUTTONUP;
				break;
			case WM_RBUTTONDOWN:
				eventType=FSMOUSEEVENT_RBUTTONDOWN;
				break;
			case WM_RBUTTONUP:
				eventType=FSMOUSEEVENT_RBUTTONUP;
				break;
			case WM_MOUSEMOVE:
				eventType=FSMOUSEEVENT_MOVE;
				break;
			}

			int lb=((wp & MK_LBUTTON)!=0);
			int mb=((wp & MK_MBUTTON)!=0);
			int rb=((wp & MK_RBUTTON)!=0);
			unsigned int shift=((wp & MK_SHIFT)!=0);
			unsigned int ctrl=((wp & MK_CONTROL)!=0);
			int mx=LOWORD(lp);
			int my=HIWORD(lp);

			if(eventType==FSMOUSEEVENT_MOVE &&
			   0<nMosBufUsed &&
			   mosBuffer[nMosBufUsed-1].eventType==FSMOUSEEVENT_MOVE &&
			   mosBuffer[nMosBufUsed-1].lb==lb &&
			   mosBuffer[nMosBufUsed-1].mb==mb &&
			   mosBuffer[nMosBufUsed-1].rb==rb &&
			   mosBuffer[nMosBufUsed-1].shift==shift &&
			   mosBuffer[nMosBufUsed-1].ctrl==ctrl)
			{
				mosBuffer[nMosBufUsed-1].mx=mx;
				mosBuffer[nMosBufUsed-1].my=my;
				break;
			}

			mosBuffer[nMosBufUsed].eventType=eventType;
			mosBuffer[nMosBufUsed].lb=lb;
			mosBuffer[nMosBufUsed].mb=mb;
			mosBuffer[nMosBufUsed].rb=rb;
			mosBuffer[nMosBufUsed].shift=shift;
			mosBuffer[nMosBufUsed].ctrl=ctrl;
			mosBuffer[nMosBufUsed].mx=mx;
			mosBuffer[nMosBufUsed].my=my;
			nMosBufUsed++;
		}
		break;

	default:
		return DefWindowProc(hWnd,msg,wp,lp);
	}
	return 1;
}

static void YsSetPixelFormat(HDC hDC)
{
	static PIXELFORMATDESCRIPTOR pfd=
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL|PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		24,
		0,0,0,0,0,0,
		0,
		0,
		0,
		0,0,0,0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0,0,0
	};

	int pfm;
	pfm=ChoosePixelFormat(hDC,&pfd);
	if(pfm!=0)
	{
		if(SetPixelFormat(hDC,pfm,&pfd)!=FALSE)
		{
			fsWin32Internal.hPlt=YsCreatePalette(hDC);
			SelectPalette(hDC,fsWin32Internal.hPlt,FALSE);
			RealizePalette(hDC);
			return;
		}
	}

	MessageBoxA(NULL,"Error In YsSetPixelFormat.",NULL,MB_OK);
	exit(1);
}

static unsigned char YsPalVal(unsigned long n,unsigned bit,unsigned sft)
{
	unsigned long msk;
	n>>=sft;
	msk=(1<<bit)-1;
	n&=msk;
	return (unsigned char)(n*255/msk);
}

/* ? lp=LocalAlloc(LMEM_FIXED,sizeof(LOGPALETTE)+n*sizeof(PALETTEENTRY)); */
/* ? LocalFree(lp); */

static HPALETTE YsCreatePalette(HDC dc)
{
	HPALETTE neo;
    PIXELFORMATDESCRIPTOR pfd;
    LOGPALETTE *lp;
    int n,i;

    n=GetPixelFormat(dc);
    DescribePixelFormat(dc,n,sizeof(PIXELFORMATDESCRIPTOR),&pfd);

    if(pfd.dwFlags & PFD_NEED_PALETTE)
    {
        n=1<<pfd.cColorBits;
        lp=(LOGPALETTE *)malloc(sizeof(LOGPALETTE)+n*sizeof(PALETTEENTRY));
        lp->palVersion=0x300;
        lp->palNumEntries=(WORD)n;
        for (i=0; i<n; i++)
        {
            lp->palPalEntry[i].peRed  =YsPalVal(i,pfd.cRedBits,pfd.cRedShift);
            lp->palPalEntry[i].peGreen=YsPalVal(i,pfd.cGreenBits,pfd.cGreenShift);
            lp->palPalEntry[i].peBlue =YsPalVal(i,pfd.cBlueBits,pfd.cBlueShift);
            lp->palPalEntry[i].peFlags=0;
        }

        neo=CreatePalette(lp);
		free(lp);
	    return neo;
    }
	return NULL;
}

static void InitializeOpenGL(HWND wnd)
{
	RECT rect;

    GetClientRect(wnd,&rect);

    glClearColor(1.0F,1.0F,1.0F,0.0F);
    glClearDepth(1.0F);
	glDisable(GL_DEPTH_TEST);

	glViewport(0,0,rect.right,rect.bottom);

    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,(float)rect.right-1,(float)rect.bottom-1,0,-1,1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glShadeModel(GL_FLAT);
	glPointSize(1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glColor3ub(0,0,0);
}

void FsChangeToProgramDir(void)
{
	char fn[MAX_PATH];
	GetModuleFileNameA(NULL,fn,MAX_PATH);

	const int l=strlen(fn);
	int i;
	for(i=l-1; i>=0; i--)
	{
		if(fn[i]=='\\' || fn[i]=='/')
		{
			fn[i]=0;
			break;
		}
	}

	_chdir(fn);
}

void FsClearEventQueue(void)
{
	for(;;)
	{
		int checkAgain=0;

		FsPollDevice();

		int lb,mb,rb,mx,my;
		while(FSMOUSEEVENT_NONE!=FsGetMouseEvent(lb,mb,rb,mx,my) ||
		      FSKEY_NULL!=FsInkey() ||
		      0!=FsInkeyChar() ||
		      0!=FsCheckWindowExposure())
		{
			checkAgain=1;
		}

		if(0!=lb || 0!=rb || 0!=mb)
		{
			checkAgain=1;
		}

		if(0!=FsCheckKeyHeldDown())
		{
			checkAgain=1;
		}

		if(0==checkAgain)
		{
			break;
		}

		FsSleep(50);
	}
}

int FsCheckKeyHeldDown(void)
{
	int keyCode;
	for(keyCode=FSKEY_NULL+1; keyCode<FSKEY_NUM_KEYCODE; keyCode++)
	{
		if(0!=FsGetKeyState(keyCode))
		{
			return 1;
		}
	}
	return 0;
}


////////////////////////////////////////////////////////////



FsWin32KeyMapper::FsWin32KeyMapper()
{
	int i;

	mapVKtoFSKEY=new int [FS_NUM_VK];
	mapFSKEYtoVK=new int [FSKEY_NUM_KEYCODE];

	for(i=0; i<FS_NUM_VK; i++)
	{
		mapVKtoFSKEY[i]=FSKEY_NULL;
	}
	for(i=0; i<FSKEY_NUM_KEYCODE; i++)
	{
		mapFSKEYtoVK[i]=0;
	}

	AddKeyMapping(FSKEY_SPACE,               VK_SPACE);
	AddKeyMapping(FSKEY_0,                   '0');
	AddKeyMapping(FSKEY_1,                   '1');
	AddKeyMapping(FSKEY_2,                   '2');
	AddKeyMapping(FSKEY_3,                   '3');
	AddKeyMapping(FSKEY_4,                   '4');
	AddKeyMapping(FSKEY_5,                   '5');
	AddKeyMapping(FSKEY_6,                   '6');
	AddKeyMapping(FSKEY_7,                   '7');
	AddKeyMapping(FSKEY_8,                   '8');
	AddKeyMapping(FSKEY_9,                   '9');
	AddKeyMapping(FSKEY_A,                   'A');
	AddKeyMapping(FSKEY_B,                   'B');
	AddKeyMapping(FSKEY_C,                   'C');
	AddKeyMapping(FSKEY_D,                   'D');
	AddKeyMapping(FSKEY_E,                   'E');
	AddKeyMapping(FSKEY_F,                   'F');
	AddKeyMapping(FSKEY_G,                   'G');
	AddKeyMapping(FSKEY_H,                   'H');
	AddKeyMapping(FSKEY_I,                   'I');
	AddKeyMapping(FSKEY_J,                   'J');
	AddKeyMapping(FSKEY_K,                   'K');
	AddKeyMapping(FSKEY_L,                   'L');
	AddKeyMapping(FSKEY_M,                   'M');
	AddKeyMapping(FSKEY_N,                   'N');
	AddKeyMapping(FSKEY_O,                   'O');
	AddKeyMapping(FSKEY_P,                   'P');
	AddKeyMapping(FSKEY_Q,                   'Q');
	AddKeyMapping(FSKEY_R,                   'R');
	AddKeyMapping(FSKEY_S,                   'S');
	AddKeyMapping(FSKEY_T,                   'T');
	AddKeyMapping(FSKEY_U,                   'U');
	AddKeyMapping(FSKEY_V,                   'V');
	AddKeyMapping(FSKEY_W,                   'W');
	AddKeyMapping(FSKEY_X,                   'X');
	AddKeyMapping(FSKEY_Y,                   'Y');
	AddKeyMapping(FSKEY_Z,                   'Z');
	AddKeyMapping(FSKEY_ESC,                 VK_ESCAPE);
	AddKeyMapping(FSKEY_F1,                  VK_F1);
	AddKeyMapping(FSKEY_F2,                  VK_F2);
	AddKeyMapping(FSKEY_F3,                  VK_F3);
	AddKeyMapping(FSKEY_F4,                  VK_F4);
	AddKeyMapping(FSKEY_F5,                  VK_F5);
	AddKeyMapping(FSKEY_F6,                  VK_F6);
	AddKeyMapping(FSKEY_F7,                  VK_F7);
	AddKeyMapping(FSKEY_F8,                  VK_F8);
	AddKeyMapping(FSKEY_F9,                  VK_F9);
	AddKeyMapping(FSKEY_F10,                 VK_F10);
	AddKeyMapping(FSKEY_F11,                 VK_F11);
	AddKeyMapping(FSKEY_F12,                 VK_F12);
	AddKeyMapping(FSKEY_PRINTSCRN,           0 /* Unassignable */);
	AddKeyMapping(FSKEY_SCROLLLOCK,          VK_SCROLL);
	AddKeyMapping(FSKEY_PAUSEBREAK,          VK_PAUSE);
	AddKeyMapping(FSKEY_TILDA,               VK_OEM_3);
	AddKeyMapping(FSKEY_MINUS,               VK_OEM_MINUS);
	AddKeyMapping(FSKEY_PLUS,                VK_OEM_PLUS);
	AddKeyMapping(FSKEY_BS,                  VK_BACK);
	AddKeyMapping(FSKEY_TAB,                 VK_TAB);
	AddKeyMapping(FSKEY_LBRACKET,            VK_OEM_4);
	AddKeyMapping(FSKEY_RBRACKET,            VK_OEM_6);
	AddKeyMapping(FSKEY_BACKSLASH,           VK_OEM_5);
	AddKeyMapping(FSKEY_CAPSLOCK,            VK_CAPITAL);
	AddKeyMapping(FSKEY_SEMICOLON,           VK_OEM_1);
	AddKeyMapping(FSKEY_SINGLEQUOTE,         VK_OEM_7);
	AddKeyMapping(FSKEY_ENTER,               VK_RETURN);
	AddKeyMapping(FSKEY_SHIFT,               VK_SHIFT);
	AddKeyMapping(FSKEY_COMMA,               VK_OEM_COMMA);
	AddKeyMapping(FSKEY_DOT,                 VK_OEM_PERIOD);
	AddKeyMapping(FSKEY_SLASH,               VK_OEM_2);
	AddKeyMapping(FSKEY_CTRL,                VK_CONTROL);
	AddKeyMapping(FSKEY_ALT,                 VK_MENU);
	AddKeyMapping(FSKEY_INS,                 VK_INSERT);
	AddKeyMapping(FSKEY_DEL,                 VK_DELETE);
	AddKeyMapping(FSKEY_HOME,                VK_HOME);
	AddKeyMapping(FSKEY_END,                 VK_END);
	AddKeyMapping(FSKEY_PAGEUP,              VK_PRIOR);
	AddKeyMapping(FSKEY_PAGEDOWN,            VK_NEXT);
	AddKeyMapping(FSKEY_UP,                  VK_UP);
	AddKeyMapping(FSKEY_DOWN,                VK_DOWN);
	AddKeyMapping(FSKEY_LEFT,                VK_LEFT);
	AddKeyMapping(FSKEY_RIGHT,               VK_RIGHT);
	AddKeyMapping(FSKEY_NUMLOCK,             VK_NUMLOCK);
	AddKeyMapping(FSKEY_TEN0,                VK_NUMPAD0);
	AddKeyMapping(FSKEY_TEN1,                VK_NUMPAD1);
	AddKeyMapping(FSKEY_TEN2,                VK_NUMPAD2);
	AddKeyMapping(FSKEY_TEN3,                VK_NUMPAD3);
	AddKeyMapping(FSKEY_TEN4,                VK_NUMPAD4);
	AddKeyMapping(FSKEY_TEN5,                VK_NUMPAD5);
	AddKeyMapping(FSKEY_TEN6,                VK_NUMPAD6);
	AddKeyMapping(FSKEY_TEN7,                VK_NUMPAD7);
	AddKeyMapping(FSKEY_TEN8,                VK_NUMPAD8);
	AddKeyMapping(FSKEY_TEN9,                VK_NUMPAD9);
	AddKeyMapping(FSKEY_TENDOT,              VK_DECIMAL);
	AddKeyMapping(FSKEY_TENSLASH,            VK_DIVIDE);
	AddKeyMapping(FSKEY_TENSTAR,             VK_MULTIPLY);
	AddKeyMapping(FSKEY_TENMINUS,            VK_SUBTRACT);
	AddKeyMapping(FSKEY_TENPLUS,             VK_ADD);
	AddKeyMapping(FSKEY_TENENTER,            0 /* Unassignable */);
}

void FsWin32KeyMapper::AddKeyMapping(int fskey,int vk)
{
	if(fskey<0 || FSKEY_NUM_KEYCODE<=fskey)
	{
		printf("FSKEY is out of range\n");
		exit(1);
	}
	if(vk<0 || FS_NUM_VK<=vk)
	{
		printf("VK is out of range\n");
		exit(1);
	}

	mapVKtoFSKEY[vk]=fskey;
	mapFSKEYtoVK[fskey]=vk;
}

FsWin32KeyMapper::~FsWin32KeyMapper()
{
	delete [] mapFSKEYtoVK;
	delete [] mapVKtoFSKEY;
}

int FsWin32KeyMapper::VkToFsKey(int vk)
{
	if(0<=vk && vk<FS_NUM_VK)
	{
		return mapVKtoFSKEY[vk];
	}
	return FSKEY_NULL;
}

int FsWin32KeyMapper::FsKeyToVk(int fsKey)
{
	if(0<=fsKey && fsKey<FSKEY_NUM_KEYCODE)
	{
		return mapFSKEYtoVK[fsKey];
	}
	return 0;
}

////////////////////////////////////////////////////////////

// Copied from fswin32winmain.cpp

static char *YsStrtok(char **src);
static int YsArguments(int *ac,char *av[],int mxac,char *src);

extern int main(int ac,char *av[]);

int PASCAL WinMain(HINSTANCE inst,HINSTANCE,LPSTR param,int)
{
	int ac;
	static char *av[512],tmp[4096],prog[MAX_PATH];

	strcpy(prog,"Unknown");
	GetModuleFileNameA(inst,prog,260);

	strncpy(tmp,param,256);
	av[0]=prog;

	YsArguments(&ac,av+1,510,tmp);

	return main(ac+1,av);
}




static int YsArguments(int *ac,char *av[],int mxac,char *src)
{
	char *arg;
	int len;

	while(*src==' ' || *src=='\t')
	{
		src++;
	}

	len=strlen(src)-1;
	while(len>=0 && isprint(src[len])==0)
	{
		src[len]=0;
		len--;
	}

	*ac=0;
	while((arg=YsStrtok(&src))!=NULL && *ac<mxac)
	{
		av[*ac]=arg;
		(*ac)++;
	}

	if(*ac<mxac)  // 2007/06/22 Added this check.
	{
		av[*ac]=NULL;
	}

	return *ac;
}

static char *YsStrtok(char **src)
{
	char *r;
	switch(**src)
	{
	case 0:
		return NULL;
	case '\"':
		(*src)++;
		r=(*src);
		while((**src)!='\"' && (**src)!=0)
		{
			(*src)++;
		}
		break;
	default:
		r=(*src);
		while((**src)!=' ' && (**src)!='\t' && (**src)!=0)
		{
			(*src)++;
		}
		break;
	}
	if((**src)!=0)
	{
		(**src)=0;
		(*src)++;
	}
	while((**src)!=0 && ((**src)==' ' || (**src)=='\t'))
	{
		(*src)++;
	}
	return r;
}
