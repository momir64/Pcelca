#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <ctime>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <condition_variable>
#pragma warning(disable:4996)

#define GORNJA_IVICA 103
#define DONJA_IVICA  472
#define LEVA_IVICA 1
#define DESNA_IVICA 959

//#define KRAJ 875
#define DEBLJINA 200
#define KRAJ DESNA_IVICA - 1
#define POCETAK KRAJ - DEBLJINA
#define VISINA DONJA_IVICA - GORNJA_IVICA
#define SIRINA DESNA_IVICA - LEVA_IVICA
#define HOR_RAZMAK 5
#define MIN_SKOK 120
#define RAZMAK 5
#define BR_EL (VISINA) / RAZMAK

#define PCELICAX1 455
#define PCELICAX2 485

#define TOLERANCIJA 30
#define BR_ELEMENATA 8

#define min_f(a, b, c)  (fminf(a, fminf(b, c)))
#define max_f(a, b, c)  (fmaxf(a, fmaxf(b, c)))

typedef struct {
	int h;
	int s;
	int v;
	int dh;
	int ds;
	int dv;
	int e1;
	int e2;
	int offset;
	float procenat;

}Proizvod;

cv::Mat take_screenshot(int top, int left, int bottom, int right);
int rgb_to_hsv(int ir, int ig, int ib);
void control();
void findBee();
void findPipe();
void uzmi_ss();
void PripremiMat();
void gotoy(int y = 15);
bool proveraUslova(int h, int j);

std::condition_variable waitsc, waitb, waitp;
std::mutex mtx, mtxf;

int gornja_granica = 0;
int donja_granica = 300, donja_granica2;
int trenutni = 0;
int naso = 0;
bool autofind = true;
bool firsttime = true;
bool v_niz[100];

int visinab = 0;
int trazi_puta = 6;
int brojac_prosao = 0;
int hue[256][256][256];
int pcelica_sablon[14] = { 359, 359, 200, 210, 358, 358, 358, 359, 240, 240, 210, 240, 0, 0 };
double delay = 1;
double V, V0, h, g, dt = 0;
double Vmx = 0, Vmn = 0;
bool controling = false, foundb = true, gotovp = true, gotovb = true, prviput = true;

HDC hDC_Desktop = GetDC(0);
HBRUSH blueBrush = CreateSolidBrush(RGB(0, 0, 255));
RECT rect = { 380, 0, 440, 0 }, rectb{ 0, 0, DESNA_IVICA, 0 };
HANDLE ConsoleOutputHandle = GetStdHandle(STD_OUTPUT_HANDLE);
FILE *f;
cv::Mat slika(VISINA, DESNA_IVICA - LEVA_IVICA, CV_8UC3), slikab(VISINA, DESNA_IVICA - LEVA_IVICA, CV_8UC1);
cv::UMat gpuslika(VISINA, DESNA_IVICA - LEVA_IVICA, CV_8UC3), gpuslika2(VISINA, DESNA_IVICA - LEVA_IVICA, CV_8UC3);
cv::UMat gpuslikab(VISINA, DESNA_IVICA - LEVA_IVICA, CV_8UC1), gpuslikab2(VISINA, DESNA_IVICA - LEVA_IVICA, CV_8UC1), elementgpu[8], element2gpu[8];
cv::Scalar hsv_low[8], hsv_high[8];

std::chrono::high_resolution_clock::time_point pocetak;

Proizvod proizvodi[8] = {
		{ 318,  80,  39,  9,  55,  60,  1,   4,   4,  10 },   //plazma
		{  90,  51,  28,  9, 100,  20,  2,   5,   3,  10 },   //jacobs
		{ 251,  31,  69,  1,  23,  20,  5,   0,   7,  10 },	 //milka
		{  12,  54,  40,  3,  59,  81,  2,   1,  36,  10 },   //pepsi
		{ 203,  37,  89,  5,  25,  22,  1,   3,  30,  10 },   //milky
		{  49,  87,  96,  1,  33,   7,  1,   9,  36,  10 },	 //chips 
		{  56, 100, 100,  1,  98,  33,  5,  11,  50,  10 },   //mer
		{ 248,  27,  69,  0,  24,  39,  1,  10,  53,  10 }    //merix
};

int main() {
	std::ios_base::sync_with_stdio(false);
	PripremiMat();
	cv::ocl::setUseOpenCL(true);
	for (int i = 0; i < 8; i++) {
		hsv_low[i] = cv::Scalar(proizvodi[i].h / 2.0 - proizvodi[i].dh, proizvodi[i].s * 255.0 / 100.0 - proizvodi[i].ds, proizvodi[i].v * 255.0 / 100.0 - proizvodi[i].dv);
		hsv_high[i] = cv::Scalar(proizvodi[i].h / 2.0 + proizvodi[i].dh, proizvodi[i].s * 255.0 / 100.0 + proizvodi[i].ds, proizvodi[i].v * 255.0 / 100.0 + proizvodi[i].dv);
		cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * proizvodi[i].e1 + 1, 2 * proizvodi[i].e1 + 1), cv::Point(proizvodi[i].e1, proizvodi[i].e1)).copyTo(elementgpu[i]);
		cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * proizvodi[i].e2 + 1, 2 * proizvodi[i].e2 + 1), cv::Point(proizvodi[i].e2, proizvodi[i].e2)).copyTo(element2gpu[i]);
	}
	std::thread take_ss(uzmi_ss), findp(findPipe), findb(findBee), controler(control);
	controler.detach();
	take_ss.detach();
	findp.detach();
	findb.detach();
	while (1) {
		V0 = -319;
		g = -1107;
		V = 1.5 * V0;
		h = VISINA / 2.0;
		while (GetAsyncKeyState(VK_LBUTTON)) Sleep(1);
		while (!GetAsyncKeyState(VK_LBUTTON))Sleep(1);

		std::chrono::high_resolution_clock::time_point pt = std::chrono::high_resolution_clock::now();
		while (controling) {
			std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
			dt = (std::chrono::duration_cast<std::chrono::duration<double>>(t - pt)).count();
			V = V - g * dt;
			h = max(min(h + V * dt, (double)VISINA), 0.0);
			//mtxf.lock();
			if (foundb) {
				h = h * 0.9 + visinab * 0.1;
				foundb = false;
			}
			//mtxf.unlock();

			if ((h + 28 + V * 0.02 >= donja_granica2) && controling && h >= 70) {
				mouse_event(MOUSEEVENTF_LEFTDOWN | MOUSEEVENTF_ABSOLUTE, 50, 50, 0, 0);
				Sleep(1);
				mouse_event(MOUSEEVENTF_LEFTUP | MOUSEEVENTF_ABSOLUTE, 50, 50, 0, 0);
				V = V0;
			}

			pt = t;
			rect.top = (int)(round(h)) - 15 + GORNJA_IVICA;
			rect.bottom = rect.top + 30;
		}
	}
}

void control() {
	while (true) {
		while (GetAsyncKeyState(' '))Sleep(10);
		while (!GetAsyncKeyState(' '))Sleep(10);
		controling = !controling;
		brojac_prosao = 0;
		pocetak = std::chrono::high_resolution_clock::now();
		delay = 0.6;
		donja_granica2 = 300;
		donja_granica = 300;
	}
}

void PripremiMat() {
	for (int r = 0; r < 256; r++)
		for (int g = 0; g < 256; g++)
			for (int b = 0; b < 256; b++)
				hue[r][g][b] = rgb_to_hsv(r, g, b);
}

int rgb_to_hsv(int ir, int ig, int ib) {
	int h = 0;
	float r = static_cast<float>(ir) / 255.0f, g = static_cast<float>(ig) / 255.0f, b = static_cast<float>(ib) / 255.0f;
	float cmax = max_f(r, g, b);
	float diff = cmax - min_f(r, g, b);
	if (!diff)
		h = 0;
	else if (cmax == r)
		h = (int)(round(fmod((60.0 * ((g - b) / diff) + 360.0f), 360.0)));
	else if (cmax == g)
		h = (int)(round(fmod((60.0 * ((b - r) / diff) + 120.0f), 360.0)));
	else if (cmax == b)
		h = (int)(round(fmod((60.0 * ((r - g) / diff) + 240.0f), 360.0)));
	return h;
}

void uzmi_ss() {
	std::unique_lock<std::mutex> lck(mtx);
	std::chrono::high_resolution_clock::time_point pt = std::chrono::high_resolution_clock::now();
	while (true) {
		std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
		gotoy();
		std::cout << std::setw(10) << (std::chrono::duration_cast<std::chrono::duration<double>>(t - pt)).count() << std::setw(20) << brojac_prosao;
		gotoy(0);
		pt = t;
		//FillRect(hDC_Desktop, &rectb, blueBrush); //ispis granice nemoj b da te zbuni
		//FillRect(hDC_Desktop, &rect, blueBrush); //ispis pcelice 

		while (!gotovb)
			waitb.wait(lck);
		while (!gotovp)
			waitp.wait(lck);

		slika = take_screenshot(GORNJA_IVICA, LEVA_IVICA, DONJA_IVICA, DESNA_IVICA);
		gotovp = false;
		gotovb = false;
		waitsc.notify_all();
	}
}

void findPipe() {
	std::chrono::high_resolution_clock::time_point nasot = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point pt = std::chrono::high_resolution_clock::now();
	std::unique_lock<std::mutex> lck(mtx);
	while (true) {
		while (gotovp)
			waitsc.wait(lck);

		cv::cvtColor(slika, gpuslika, cv::COLOR_BGR2HSV);
		cv::inRange(gpuslika, hsv_low[trenutni], hsv_high[trenutni], gpuslikab);
		cv::erode(gpuslikab, gpuslikab2, elementgpu[trenutni]);
		cv::dilate(gpuslikab2, gpuslikab, element2gpu[trenutni]);

		if (GetAsyncKeyState('R'))
			autofind = true,
			brojac_prosao = 0,
			delay = 0.9, //delay=0.85
			pocetak = std::chrono::high_resolution_clock::now();

		//for (int j = 0; j < 8; j++)
		//	if (GetAsyncKeyState(49 + j))
		//		trenutni = j;

		int x1;
		gpuslikab.copyTo(slikab);
		float procenat = 0;
		for (int j = 0; j < BR_EL; j++) {
			x1 = POCETAK;
			while (x1 < KRAJ && !slikab.at<uchar>(j * RAZMAK, x1))
				x1++;
			if (x1 != KRAJ) {
				v_niz[j] = 1;
				procenat++;
			}
			else
				v_niz[j] = 0;
		}

		if (procenat / ((float)BR_EL / 100.0) >= proizvodi[trenutni].procenat) {
			autofind = false;
			naso++;
			if (naso == trazi_puta) {
				int maax = 0, brojac, poc, kraj, p;
				for (p = 0; p < BR_EL; p++) {
					brojac = p;
					do
						brojac++;
					while (brojac < BR_EL && !v_niz[brojac]);
					if (brojac - p > maax) {
						maax = brojac - p;
						poc = p;
						kraj = brojac - 1;
					}
					p = brojac - 1;
				}

				if (kraj == BR_EL - 1) {
					gornja_granica = 0;
					for (x1 = POCETAK; x1 < KRAJ; x1 += HOR_RAZMAK) {
						for (p = poc * RAZMAK; p < (poc - 1) * RAZMAK && !slikab.at<uchar>(p, x1); p--);
						gornja_granica = max(p, gornja_granica);
					}
					donja_granica = gornja_granica + proizvodi[trenutni].offset + MIN_SKOK;
				}
				else {
					donja_granica = VISINA;
					for (x1 = POCETAK; x1 < KRAJ; x1 += HOR_RAZMAK) {
						for (p = kraj * RAZMAK; p < (kraj + 1) * RAZMAK && !slikab.at<uchar>(p, x1); p++);
						donja_granica = min(p, donja_granica);
					}
					donja_granica -= proizvodi[trenutni].offset;
				}

				nasot = std::chrono::high_resolution_clock::now();
				/*f = fopen("vreme.txt", "a+");
				fprintf(f, "%lf %d\n", (std::chrono::duration_cast<std::chrono::duration<double>>(nasot-pocetak)).count(), brojac_prosao);
				fclose(f);*/
				brojac_prosao++;
				if (brojac_prosao < 10)
					delay = 0.75;
				else if (brojac_prosao == 20)
					delay = 0.65;
				else if (brojac_prosao == 25)
					delay = 0.48, trazi_puta = 4;
				else if (brojac_prosao == 30)
					delay = 0.46;
				else if (brojac_prosao == 35)
					delay = 0.44, trazi_puta = 3;
				else if (brojac_prosao == 40)
					delay = 0.43;
				else if (brojac_prosao == 45)
					delay = 0.38;
				else if (brojac_prosao == 50)
					delay = 0.35;
				else if (brojac_prosao == 55)
					delay = 0.3, trazi_puta = 2;
				else if (brojac_prosao == 60)
					delay = 0.27;
				else if (brojac_prosao == 65)
					delay = 0.26;
				else if (brojac_prosao == 70)
					delay = 0.23;

				trenutni = trenutni + 1 == 8 ? 0 : trenutni + 1;
				naso = 0;
			}
		}
		if (autofind)
			trenutni = trenutni + 1 == 8 ? 0 : trenutni + 1;

		std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
		if ((std::chrono::duration_cast<std::chrono::duration<double>>(t - nasot)).count() > delay)
			donja_granica2 = donja_granica;

		rectb.bottom = donja_granica2 + GORNJA_IVICA + 2;
		rectb.top = rectb.bottom - 4;

		pt = t;
		gotovp = true;
		waitp.notify_all();
	}
}

void gotoy(int y) {
	COORD coord;
	coord.X = 0;
	coord.Y = y;
	SetConsoleCursorPosition(ConsoleOutputHandle, coord);
	CONSOLE_CURSOR_INFO info;
	info.dwSize = 100;
	info.bVisible = FALSE;
	SetConsoleCursorInfo(ConsoleOutputHandle, &info);
}

cv::Mat take_screenshot(int top, int left, int bottom, int right) {
	int height = bottom - top, width = right - left;
	HDC hwindowDC = GetDC(NULL), hwindowCompatibleDC = CreateCompatibleDC(hwindowDC);
	HBITMAP hbwindow = CreateCompatibleBitmap(hwindowDC, width, height);
	SetStretchBltMode(hwindowCompatibleDC, 3);
	BITMAPINFOHEADER  bi;
	cv::Mat src;
	src.create(height, width, 24);
	bi.biSize = sizeof(BITMAPINFOHEADER);
	bi.biWidth = width;
	bi.biHeight = -height;
	bi.biPlanes = 1;
	bi.biBitCount = 32;
	bi.biCompression = 0L;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 0;
	bi.biYPelsPerMeter = 0;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;
	SelectObject(hwindowCompatibleDC, hbwindow);
	StretchBlt(hwindowCompatibleDC, 0, 0, width, height, hwindowDC, left, top, width, height, 0x00CC0020);
	GetDIBits(hwindowCompatibleDC, hbwindow, 0, height, src.data, (BITMAPINFO *)&bi, 0);
	DeleteDC(hwindowCompatibleDC);
	DeleteObject(hbwindow);
	return src;
}

void findBee() {
	std::unique_lock<std::mutex> lck(mtx);
	while (true) {
		while (gotovb)
			waitsc.wait(lck);

		int h, k;
		bool breaked = false;
		for (int i = 10; i < 360; i++) {
			for (int j = PCELICAX1; j < PCELICAX2 - BR_ELEMENATA; j++) {
				k = 0;
				do {
					h = hue[slika.at<cv::Vec4b>(i, k + j)[2]][slika.at<cv::Vec4b>(i, j + k)[1]][slika.at<cv::Vec4b>(i, j + k)[0]];
					k++;
					if (k == BR_ELEMENATA) {
						visinab = i;
						breaked = true;
						//mtxf.lock();
						foundb = true;
						//mtxf.unlock();
						break;
					}
				} while (proveraUslova(h, pcelica_sablon[k - 1]));
				if (breaked) break;
			}
			if (breaked) break;
		}

		gotovb = true;
		waitb.notify_all();
	}
}

bool proveraUslova(int h, int j) {
	if (j + TOLERANCIJA >= 360 || j - TOLERANCIJA < 0)
		if ((h + 180) % 360 < (j + 180) % 360 + TOLERANCIJA && (h + 180) % 360 > (j + 180) % 360 - TOLERANCIJA)
			return true;
	if (h > j - TOLERANCIJA && h < j + TOLERANCIJA)
		return true;
	return false;
}