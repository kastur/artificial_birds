#include <iostream>
#include <cmath>
#include <cstring>
#include <string>
#include "discpp.h"

void     widget  (void);
void     mysub   (int id);
void     exa     (char *cdev, int iopt);
void     exa_1   (void);
void     exa_2   (void);
void     exa_3   (void);
void     exa_4   (void);
void     exa_5   (void);
void     exa_6   (void);
void     exa_7   (void);
void     exa_8   (void);
void     exa_9   (void);
void     exa_10  (void);
void     exa_11  (void);
void     exa12_1 (void);
void     exa_12  (void);
void     exa12_2 (void);
void     exa_13  (void);
void     exa_14  (void);
void     ex6_1   (void);
void     ex10_1  (void);
void     ex10_2  (void);
void     ex10_3  (void);
void     ex11_1  (void);
void     ex13_1  (void);
void     ex13_2  (void);
void     ex13_3  (void);
void     ex13_4  (void);
void     ex14_1  (void);
void     ex14_2  (void);
  
double   zfun    (double x, double y, int iopt);

using namespace std;

double xray[300], yray[300], y1ray[300], y2ray[300], zmat[50][50];
string cl1, cl2;
int id_lis1, id_lis2, id_pbut;

Dislin g;

int main ()
{ widget ();
  return 0;
}

void widget (void)
{ int ip, ip_l, ip_r, idev = 1, iopt = 1;
  string chelp;
  
  chelp = "C++ Demo program of the graphics library DISLIN.|";
  chelp += "Author: Helmut Michels,|";
  chelp += "Max-Planck-Institut fuer Sonnensystemforschung,|";
  chelp += "D-37191 Katlenburg-Lindau, Germany";

  cl1  = "CONS   (X Window Screen)|";
  cl1 += "XWIN   (Small Window)|";
  cl1 += "GL       (OpenGL Window)|";
  cl1 += "GKSL   (GKSLIN Metafile)|";
  cl1 += "CGM    (CGM Metafile)|";
  cl1 += "PS       (PostScript File)|";
  cl1 += "EPS     (EPS File)|";
  cl1 += "PDF     (PDF File)|";
  cl1 += "WMF    (Windows Metafile)|";
  cl1 += "HPGL   (HPGL Plotfile)|";
  cl1 += "SVG     (SVG File)|";
  cl1 += "JAVA   (Java Applet File)|";
  cl1 += "GIF     (GIF File)|";
  cl1 += "TIFF   (TIFF File)|";
  cl1 += "PNG    (PNG File)|";
  cl1 += "PPM    (PPM File)|";
  cl1 += "BMP    (BMP File)|";
  cl1 += "IMAG   (Image File)";

  cl2  = "Demo of CURVE|Polar Plots|Symbols|Logarithmic Axes|";
  cl2 += "Interpolation Methods|Line Styles|Legend|";
  cl2 += "Shading Patterns (AREAF)|Vectors|";
  cl2 += "Shading Patterns (PIEGRF)|Surface Plot|";
  cl2 += "Surface Plot|Parametric Function Plot|";
  cl2 += "Map Plot|World Coastlines|";
  cl2 += "Elliptical Proj.|Azimuthal Proj.|Conical Proj.|";
  cl2 += "Bar Graphs|Pie Charts|3-D Bar Graph / Pie Chart|";
  cl2 += "Contours|Shaded Contours|";
  cl2 += "Instruction Alphabet|3-D Colour Plot|Shaded World|";
  cl2 += "Colour Tables";

  g.swgtit ("DISLIN Examples");
  g.swghlp (chelp.c_str ());
  g.swgwth (30);
  g.swgtyp ("SCROLL", "LIST");
  g.swgpop ("NOOK");
  
  ip = g.wgini ("hori");
  ip_l = g.wgbas (ip, "vert");
  ip_r = g.wgbas (ip, "vert");

  g.wglab (ip_l, "Example:");
  id_lis1 = g.wglis (ip_l, cl2.c_str (), iopt);
  
  g.wglab (ip_r, "Device:");
  id_lis2 = g.wglis (ip_r, cl1.c_str (), idev);
  
  id_pbut = g.wgpbut (ip_r, "Plot");
  g.swgcbk (id_pbut, mysub);

  g.wgquit (ip_l);
  g.wgfin ();
}  

void mysub (int id)
{ int iopt, idev;
  char  *cstr;

  if (g.getlev () != 0) return;
  if (id != id_pbut) return;
  iopt = g.gwglis (id_lis1);
  idev = g.gwglis (id_lis2);

  cstr = g.itmstr (cl1.c_str (), idev);
  exa (cstr, iopt);
  return;
}

void exa (char *cdev, int iopt)
{
  g.metafl (cdev);
  g.setpag ("da4l");
  
  switch (iopt)
  { case  1: exa_1 ();
             break;
    case  2: exa_2 ();
             break;
    case  3: exa_3 ();
             break;
    case  4: exa_4 ();
             break;
    case  5: exa_5 ();
             break;
    case  6: exa_6 ();
             break;
    case  7: exa_7 ();
             break;
    case  8: exa_8 ();
             break;
    case  9: exa_9 ();
             break;
    case 10: exa_10 ();
             break;
    case 11: exa_11 ();
             break;
    case 12: exa12_1 ();
             break;
    case 13: exa12_2 ();
             break;
    case 14: exa_12 ();
             break;
    case 15: ex13_1 ();
             break;
    case 16: ex13_2 ();
             break;
    case 17: ex13_3 ();
             break;
    case 18: ex13_4 ();
             break;
    case 19: ex10_1 ();
             break;
    case 20: ex10_2 ();
             break;
    case 21: ex10_3 ();
             break;
    case 22: ex14_1 ();
             break;
    case 23: ex14_2 ();
             break;
    case 24: ex6_1 ();
             break;
    case 25: ex11_1 ();
             break;
    case 26: exa_13 ();
             break;
    case 27: exa_14 ();
             break;
  }
}

/* >>>>>>>>>> EXA_1  <<<<<<<<<< */
void exa_1 (void)
{ int n = 100, i;
  double fpi = 3.1415926 / 180., step, x;

  step = 360. / (n - 1);

  for (i = 0; i < n; i++)
  { xray[i] = i * step;
    x = xray[i] * fpi;
    y1ray[i] = sin (x);
    y2ray[i] = cos (x);
  }

  g.disini ();
  g.pagera ();
  g.hwfont ();
  g.axspos (450, 1800);
  g.axslen (2200, 1200);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");

  g.labdig (-1, "x");
  g.ticks  (10, "xy");

  g.titlin ("Demonstration of CURVE", 1);
  g.titlin ("SIN(X), COS(X)", 3);

  g.graf   (0.0, 360.0, 0.0, 90.0, -1.0, 1.0, -1.0, 0.5);
  g.title  ();

  g.color  ("red");
  g.curve  (xray, y1ray, n);
  g.color  ("green");
  g.curve  (xray, y2ray, n);

  g.color  ("fore");
  g.dash   ();
  g.xaxgit ();
  g.disfin ();
}

/* >>>>>>>>>> EXA_2  <<<<<<<<<< */
void exa_2 (void)
{ int n = 300, m = 10, i;
  double f = 3.1415926 / 180., step, a;
  double x2[10], y2[10];
  
  step = 360. / (n - 1);
  for (i = 0; i < n; i++)
  { a = i * step * f;
    yray[i] = a;
    xray[i] = sin (5 * a);
  }

  for (i = 0; i < m; i++)
  { x2[i] = i + 1;
    y2[i] = i + 1;
  }

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();
  g.axspos (450,1800);

  g.titlin ("Polar Plots", 2);
  g.ticks  (3, "Y");
  g.axends ("NOENDS", "X");
  g.labdig (-1, "Y");
  g.axslen (1000, 1000);
  g.axsorg (1050, 900);

  g.grafp  (1.0 ,0.0, 0.2, 0.0, 30.0);
  g.curve  (xray, yray, n);
  g.htitle (50);
  g.title  ();
  g.endgrf ();

  g.labdig (-1, "X");
  g.axsorg (1050, 2250);
  g.labtyp ("VERT", "Y");
  g.grafp  (10.0, 0.0, 2.0, 0.0, 30.0);
  g.barwth (-5.0);
  g.polcrv ("FBARS");
  g.curve  (x2, y2, m);
  g.disfin();
}

/* >>>>>>>>>> EXA_3  <<<<<<<<<< */
void exa_3 (void)
{ int nl, ny, i, nxp = 0;
  static const char *ctit = "Symbols";
  char cstr[80];

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.color  ("yellow");
  g.hwfont ();

  g.height (60);

  nl = g.nlmess (ctit);
  g.messag (ctit, (2100 - nl) / 2, 200);

  g.height (50);
  g.hsymbl (120);

  ny = 150;
  for (i = 0; i < 24; i++)
  { if ((i % 4) == 0) 
    { ny  += 400;
      nxp  = 550;
    }
    else
    { nxp += 350;
    }

    nl = g.intcha (i, cstr);  
    nl = g.nlmess (cstr) / 2;
    g.messag (cstr, nxp - nl, ny + 150);
    g.symbol (i, nxp, ny);
  }
  g.disfin ();
}

/* >>>>>>>>>> EXA_4  <<<<<<<<<< */
void exa_4 (void)
{ int nya, i;
  static const char  *ctit  = "Logarithmic Scaling";
  string clab[3] = {"LOG", "FLOAT", "ELOG"};
  string cstr;

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();

  g.axslen (1400, 500);
  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");
  g.axsscl ("log", "xy");

  g. titlin (ctit, 2);

  for (i = 1; i <= 3; i++)
  { nya = 2650 - (i - 1) * 800;
    g.labdig (-1, "xy");
    if (i == 2)
    { g.labdig (1, "y");
      g.name (" ", "x");
    }

    g.axspos (500, nya);
    g.color  ("yellow");
	cstr = "Labels: ";
    cstr += clab[i-1];
    g.messag (cstr.c_str (), 600, nya - 400);
    g.color  ("fore");
    g.labels (clab[i-1].c_str (), "xy");
    g.graf(0.0, 3.0, 0.0, 1.0, -1.0, 2.0, -1.0, 1.0);

    if (i == 3)
    { g.height (50);
      g.title  ();
    }

    g.endgrf ();
  }
  g.disfin ();
}

/* >>>>>>>>>> EXA_5  <<<<<<<<<< */
void exa_5 (void)
{ int nya = 2700, i, nx, ny;
  static double
        x[] = { 0.0,  1.0,  3.0,  4.5,  6.0,  8.0,  9.0, 11.0, 12.0,
               12.5, 13.0, 15.0, 16.0, 17.0, 19.0, 20.0},
        y[] = { 2.0,  4.0,  4.5,  3.0,  1.0,  7.0,  2.0,  3.0,  5.0,
                2.0,  2.5,  2.0,  4.0,  6.0,  5.5,  4.0};
  static const char 
       *cpol[6] = {"SPLINE", "STEM", "BARS", "STAIRS", "STEP", "LINEAR"},
       *ctit    = "Interpolation Methods";

  g.setpag ("da4p");
  g.disini ();
  g.hwfont ();
  g.pagera ();
  g.incmrk (1);
  g.hsymbl (25);
  g.titlin (ctit, 1);
  g.axslen (1500, 350);
  g.setgrf ("line", "line", "line", "line");

  for (i = 0; i < 6; i++)
  { g.axspos (350, nya - i * 350);
    g.polcrv (cpol[i]);
    g.marker (0);

    g.graf (0.0, 20.0, 0.0, 5.0, 0.0, 10.0, 0.0, 5.0);
    nx = g.nxposn (1.0);
    ny = g.nyposn (8.0);
    g.color  ("yellow");
    g.messag (cpol[i], nx, ny);
    g.curve  (x, y, 16);
    g.color  ("fore");

    if (i == 5)
    { g.height (50);
      g.title  ();
    }
    g.endgrf ();
  }
  g.disfin ();
}

/* >>>>>>>>>> EXA_6  <<<<<<<<<< */
void exa_6 (void)
{ int i, ny, nx;
  static double   x[2] = {3.0, 9.0}, y[2];
  static const char *ctyp[8] = {"SOLID",  "DOT",   "DASH", "CHNDSH",
                                "CHNDOT", "DASHM", "DOTL", "DASHL"};

  g.setpag ("da4p");
  g.disini ();
  g.setvlt ("small");
  g.pagera ();
  g.center ();
  g.chncrv ("both");
  g.hwfont ();

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");

  g.titlin ("Demonstration of Curve", 1);
  g.titlin ("Line Styles", 3);

  g.graf   (0.0, 10.0, 0.0, 2.0, 0.0, 10.0, 0.0, 2.0);
  g.title  ();

  for (i = 1; i <= 8; i++)
  { y[0] = 9.5 - i;
    y[1] = 9.5 - i;
    ny = g.nyposn (y[0]);
    nx = g.nxposn (1.0);
    g.messag (ctyp[i-1], nx, ny - 20);
    g.curve  (x, y, 2);
  }
  g.disfin ();
}

/* >>>>>>>>>> EXA_7  <<<<<<<<<< */
void exa_7 (void)
{ int n = 100, i, nx, ny;
  double fpi = 3.1415926/180., step, x;
  char   cbuf[20];

  step = 360. / (n - 1);
  for (i = 0; i < n; i++)
  { xray[i] = i * step;
    x=xray[i] * fpi;
    y1ray[i] = sin (x);
    y2ray[i] = cos (x);
  }

  g.disini ();
  g.hwfont ();
  g.pagera ();

  g.axspos (450, 1800);
  g.axslen (2200, 1200);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");
  g.labdig (-1, "x");
  g.ticks  (10, "xy");

  g.titlin ("Demonstration of Curve", 1);
  g.titlin ("Legend", 3);

  g.graf   (0.0, 360.0, 0.0, 90.0, -1.0, 1.0, -1.0, 0.5);
  g.title  ();
  g.xaxgit ();

  g.chncrv ("both");
  g.curve  (xray, y1ray, n);
  g.curve  (xray, y2ray, n);

  g.legini (cbuf, 2, 7);
  nx = g.nxposn (190.0);
  ny = g.nyposn (0.75);
  g.legpos (nx, ny);
  g.leglin (cbuf, "sin (x)", 1);
  g.leglin (cbuf, "cos (x)", 2);
  g.legtit ("Legend");
  g.legend (cbuf, 3);
  g.disfin ();
}

/* >>>>>>>>>> EXA_8  <<<<<<<<<< */
void exa_8 (void)
{ int   ixp[4], iyp[4], nl, nx, nx0 = 335, ny0 = 350, ny, i, j, ii, k, iclr;
  static int  ix[4] = {0, 300, 300, 0},
              iy[4] = {0, 0, 400, 400};
  static const char *ctit = "Shading Patterns (AREAF)";
  char cstr[80];

  g.disini ();
  g.setvlt ("small");
  g.pagera ();
  g.hwfont ();

  g.height (50);
  nl = g.nlmess (ctit);
  nx = (2970 - nl) / 2;
  g.messag (ctit, nx, 200);

  iclr = 0;
  for (i = 0; i < 3; i++)
  { ny = ny0 + i * 600;
    for (j = 0; j < 6; j++)
    { nx = nx0 + j * 400;
      ii = i * 6 + j;
      g.shdpat ((long) ii);
      nl = g.intcha (ii, cstr);

      iclr = iclr % 8;
      iclr++;
      g.setclr (iclr);

      for (k = 0; k < 4; k++)
      { ixp[k] = ix[k] + nx;
        iyp[k] = iy[k] + ny;
      }
      g.areaf  (ixp, iyp, 4);

      nl  = g.nlmess (cstr);
      nx += (300 - nl) / 2;
      g.messag (cstr, nx, ny + 460);
    }
  }
  g.disfin ();
}

/* >>>>>>>>>> EXA_9  <<<<<<<<<< */
void exa_9 (void)
{ int nl, nx, ny, i;
  static int ivec[20]   = {   0,1111,1311,1421,1531,1701,1911,3111,3311,3421,
                           3531,3703,4221,4302,4413,4522,4701,5312,5502,5703};
  static const char *ctit = "Vectors";
  char cnum[80];

  g.disini ();
  g.color  ("cyan");
  g.pagera ();
  g.hwfont ();

  g.height (60);
  nl = g.nlmess (ctit);
  nx = (2970 - nl) / 2;
  g.messag (ctit, nx, 200);

  g.height (50);
  nx = 300;
  ny = 400;

  for (i = 0; i < 20; i++)
  { if (i == 10) 
    { nx += 2970 / 2;
      ny  = 400;
    }

    nl = g.intcha (ivec[i], cnum);
    nl = g.nlmess (cnum);
    g.messag (cnum, nx - nl, ny - 25);

    g.vector (nx + 100, ny, nx + 1000, ny, ivec[i]);
    ny += 160;
  }
  g.disfin ();
}

/* >>>>>>>>>> EXA_10  <<<<<<<<<< */
void exa_10 (void)
{ int i, nl;
  char cbuf[40], cstr[80];

  for (i = 0; i < 18; i++)
    xray[i] = 1.0;

  g.setpag ("da4p");
  g.disini ();
  g.setvlt ("small");
  g.pagera ();
  g.hwfont ();

  g.axspos (250, 2700);
  g.axslen (1600, 2200);
  g.titlin ("Shading Patterns (PIEGRF)", 3);
  g.height (50);

  g.legini (cbuf, 18, 2);

  for (i = 0; i < 18; i++)
  { nl = g.intcha (i, cstr);
    g.leglin (cbuf, cstr, i + 1);
  }

  g.chnpie ("both");
  g.labels ("none", "pie");
  g.piegrf (cbuf, 1, xray, 18);
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EXA_11 <<<<<<<<<< */
void exa_11 (void)
{ int n = 50 ,i, j;
  double fpi=3.1415927 / 180., step, x, y;

  step = 360. / (n - 1);
  for (i = 0; i < n; i++)
  { x = i * step;
    for (j = 0; j < n; j++)
    { y = j * step;
      zmat[i][j] = 2 * sin (x * fpi) * sin (y * fpi);
    }
  }

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();
  g.axspos (200, 2600);
  g.axslen (1800, 1800);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");
  g.name   ("Z-axis", "z");

  g.titlin ("Surface Plot (SURMAT)", 2);
  g.titlin ("F(X,Y) = 2*SIN(X)*SIN(Y)", 4);

  g.view3d (-5.0, -5.0, 4.0, "abs");
  g.graf3d (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0,
           -3.0, 3.0, -3.0, 1.0);
  g.height (50);
  g.title  ();

  g.color  ("green");
  g.surmat ((double *) zmat, 50, 50, 1, 1);
  g.disfin ();
}

/* >>>>>>>>>> EXA12_1 <<<<<<<<<< */
void exa12_1 (void)
{ int n = 50 ,i, j;
  static int ixp[4] = {200, 1999, 1999, 200},
             iyp[4] = {2600, 2600, 801, 801};

  double fpi = 3.1415927 / 180., step, x, y;

  step = 360. / (n - 1);
  for (i = 0; i < n; i++)
  { x = i * step;
    for (j = 0; j < n; j++)
    { y = j * step;
      zmat[i][j] = (double) (2 * sin (x * fpi) * sin (y * fpi));
    }
  }

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();

  g.axspos (200, 2600);
  g.axslen (1800, 1800);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");
  g.name   ("Z-axis", "z");

  g.titlin ("Surface Plot (SURMAT)", 2);
  g.titlin ("F(X,Y) = 2*SIN(X)*SIN(Y)", 4);

  g.graf3d (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0,
            -3.0, 3.0, -3.0, 1.0);
  g.height (50);
  g.title  ();
  g.shlsur ();
  g.color  ("green");
  g.surmat ((double *) zmat, n, n, 1, 1);

  g.color  ("fore");
  g.grfini (-1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0);
  g.nograf ();
  g.graf   (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0);
  g.dashl  ();
  g.grid   (1,1);
  g.grffin ();

  g.grfini (-1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0);
  g.graf   (0.0, 360.0, 0.0, 90.0, -3.0, 3.0, -3.0, 1.0);
  g.grid   (1, 1);
  g.grffin ();

  g.grfini (-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0, 1.0);
  g.shdpat (7L);
  g.solid  ();
  g.areaf  (ixp, iyp, 4);
  g.grffin ();
  g.disfin ();
}

/* >>>>>>>>>> EXA12_2 <<<<<<<<<< */
void exa12_2 (void)
{ double p = 3.14159, step;
  
   g.setpag ("da4p");
   g.disini ();
   g.pagera ();
   g.hwfont ();
   g.axspos (200, 2400);
   g.axslen (1800, 1800);
   g.intax  ();

   g.titlin ("Surface Plot of the Parametric Function", 2);
   g.titlin ("[COS(t)+(3+COS(u)), SIN(t)*(3+COS(u)), SIN(u)]", 4);

   g.name   ("X-axis", "x");
   g.name   ("Y-axis", "y");
   g.name   ("Z-axis", "z");

   g.vkytit (-300);
   g.zscale (-1.0, 1.0);
   g.graf3d (-4.0, 4.0, -4.0, 1.0, -4.0, 4.0, -4.0, 1.0,
	     -3.0, 3.0, -3.0, 1.0);
   g.height (40);
   g.title  ();

   g.surmsh ("on");
   step = (double) (2 * 3.14159 / 30.);
   g.surfcp (zfun, 0.0, 2 * p, step, 0.0, 2 * p, step);
   g.disfin ();
}

double zfun (double x, double y, int iopt)
{ double v;
  if (iopt == 1)
    v = cos ((double) x) * (3. + cos ((double) y));
  else if (iopt == 2)
    v = sin ((double) x) * (3. + cos ((double) y));
  else
    v = sin ((double) y);

  return v;
}

/* >>>>>>>>>> EXA_12 <<<<<<<<<< */
void exa_12 (void)
{ int i, nxp, nyp;
  double  xp, yp;
  static double  xc[9] = {-22.0, 18.0, 37.5, 0.0, 2.5, 12.5, 23.5, -3.75,
                          14.25},
                 yc[9] = {64.0, 59.6, 56.0, 51.5, 48.5, 42.0, 38.0, 40.3,
                          50.1};
  static const char *cstr[9] = 
           {"REYKJAVIK", "STOCKHOLM", "MOSKAU", "LONDON",
            "PARIS", "ROM", "ATHEN", "MADRID", "PRAG"};

  g.disini ();
  g.pagera ();
  g.hwfont ();

  g.axspos (500, 1850);
  g.axslen (2200, 1400);

  g.labdig (-1, "xy");
  g.ticks  (1, "xy");
  g.name   ("Longitude", "x");
  g.name   ("Latitude", "y");

  g.titlin ("Map Plot", 3);
  g.incmrk (-1);

  g.labels ("map", "xy");
  g.projct ("lambert");
  g.frame  (3);
  g.grafmp (-40.0, 60.0, -40.0, 20.0, 35.0, 70.0, 40.0, 10.0);

  g.color  ("green");
  g.world  ();
  g.color  ("fore");
  g.curvmp (xc, yc, 9);

  for (i = 0; i < 9; i++)
  { g.pos2pt (xc[i], yc[i], &xp, &yp);
    nxp = (int) (xp + 30);
    nyp = (int) yp;
    g.messag (cstr[i], nxp, nyp);
  }

  g.gridmp (1, 1);
  g.height (50);
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EX13_1 <<<<<<<<<< */
void ex13_1 (void)
{ g.setpag ("da4l");
  g.disini ();
  g.pagera ();
  g.hwfont ();

  g.frame  (3);
  g.axspos (400, 1850);
  g.axslen (2400, 1400);

  g.name   ("Longitude", "x");
  g.name   ("Latitude", "y");
  g.titlin ("World Coastlines and Lakes", 3);

  g.labels ("map", "xy");
  g.grafmp (-180.0, 180.0, -180.0, 90.0, -90.0, 90.0, -90.0, 30.0);

  g.gridmp (1, 1);
  g.color  ("green");
  g.world  ();
  g.color  ("fore");

  g.height (50);
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EX13_2 <<<<<<<<<< */
void ex13_2 (void)
{ int nya, i;
  string cproj[3] = {"Sanson", "Winkel", "Hammer"};
  string ctit;

  g.setpag ("da4p");
  g.disini ();
  g.hwfont ();
  g.pagera ();

  g.height (40);
  g.axslen (1600, 750);

  nya = 3850;
  for (i = 0; i < 3; i++)
  { nya = nya - 950;
    g.axspos (250, nya);

    g.projct (cproj[i].c_str ());
    g.noclip ();
    g.grafmp (-180.0, 180.0, -180.0, 30.0, -90.0, 90.0, -90.0, 15.0);

	ctit = "Elliptical Projection of ";
	ctit += cproj[i];
    g.titlin (ctit.c_str (), 4);
    g.title  ();
    g.color  ("green");
    g.world  ();
    g.color  ("fore");

    g.gridmp (1, 1);
    g.endgrf ();
  }
  g.disfin ();
}

/* >>>>>>>>>> EX13_3 <<<<<<<<<< */
void ex13_3 (void)
{ int    nl, nx, i;
  static int   nxa[4]  = {200, 1150, 200, 1150},
               nya[4]  = {1600, 1600, 2700, 2700};
  static double xpol[4] = {0.0, 0.0, 0.0, 0.0},
                ypol[4] = {0.0, 45.0, 90.0, -45.0};
  static const char *ctit = "Azimuthal Lambert Projections";

  g.setpag ("da4p");
  g.disini ();
  g.hwfont ();
  g.pagera ();

  g.height (50);
  nl = g.nlmess (ctit);
  nx = (2250 - nl) / 2;
  g.messag (ctit, nx, 300);

  g.axslen (900, 900);
  g.projct ("lambert");

  for(i = 0; i < 4; i++)
  { g.axspos (nxa[i], nya[i]);
    g.mappol (xpol[i], ypol[i]);
    g.grafmp (-180.0, 180.0, -180.0, 30.0, -90.0, 90.0, -90.0, 30.0);

    g.color  ("green");
    g.world  ();
    g.color  ("fore");
    g.gridmp (1, 1);
    g.endgrf ();
  }
  g.disfin ();
}

/* >>>>>>>>>> EX13_4 <<<<<<<<<< */
void ex13_4 (void)
{ int n = 32, inray[32], icray[32], i;
  long ipray[32];

  for (i = 0; i < 32; i++)
  { inray[i] = i + 1;
    ipray[i] = 0;
    icray[i] = 1;
  }

  g.setpag ("da4p");
  g.disini ();
  g.setvlt ("small");
  g.pagera ();
  g.hwfont ();

  g.intax  ();
  g.ticks  (1, "xy");
  g.frame  (3);
  g.axslen (1600, 2200);
  g.axspos (400, 2700);

  g.name   ("Longitude", "x");
  g.name   ("Latitude", "y");
  g.titlin ("Conformal Conic Projection", 3);

  g.labels ("map", "xy");
  g.projct ("conf");
  g.grafmp (-10.0, 30.0, -10.0, 5.0, 35.0, 70.0, 35.0, 5.0);

  g.gridmp (1, 1);
  g.shdeur (inray, ipray, icray, n);

  g.height (50);
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EX14_1 <<<<<<<<<< */
void ex14_1 (void)
{ int n = 50, i, j;
  double   fpi = 3.14159 / 180., step, x, y, zlev;

  step = 360. / (n - 1);

  for (i = 0; i < n; i++)
  { xray[i] = i * step;
    yray[i] = i * step;
  }

  for (i = 0; i < n; i++)
  { for (j = 0; j < n; j++)
    { x = xray[i] * fpi;
      y = yray[j] * fpi;    
      zmat[i][j] = 2 * sin (x) * sin (y);
    }
  }

  g.setpag ("da4p");
  g.disini ();
  g.complx ();
  g.pagera ();

  g.titlin ("Contour Plot", 1);
  g.titlin ("F(X,Y) = 2 * SIN(X) * SIN(Y)", 3);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");

  g.intax  ();
  g.axspos (450, 2670);
  g.graf   (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0);

  g.height (30);
  for (i = 0; i < 9; i++)
  { zlev = -2. + i * 0.5;
    g.setclr ((i + 1) * 25);
    if (i == 4)
      g.labels ("none", "contur"); 
    else
      g.labels ("float", "contur");

    g.contur  (xray, n, yray, n, (double *) zmat, zlev);
  }

  g.height (50);
  g.color  ("fore");
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EX14_2 <<<<<<<<<< */
void ex14_2 (void)
{ int    n = 50, i, j;
  double zlev[12];
  double step, x, y;

  step = 1.6 / (n - 1);
  for (i = 0; i < n; i++)
  { x = 0.0 + i * step;
    xray[i] = x;
    for (j = 0; j < n; j++)
    { y = 0.0 + j * step;
      yray[j] = y;
      zmat[i][j] = (x * x - 1.) * (x * x - 1.) + 
                   (y * y - 1.) * (y * y - 1.);
    }
  }

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();

  g.mixalf ();
  g.titlin ("Shaded Contour Plot", 1);
  g.titlin ("F(X,Y) = (X[2$ - 1)[2$ + (Y[2$ - 1)[2$", 3);
  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");

  g.shdmod ("poly", "contur");
  g.axspos (450, 2670);
  g.graf   (0.0, 1.6, 0.0, 0.2, 0.0, 1.6, 0.0, 0.2);

  for (i = 1; i <= 12; i++)
    zlev[12-i] = 0.1 + (i - 1) * 0.1;

  g.conshd (xray, n, yray, n, (double *) zmat, zlev, 12);

  g.height (50);
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EX6_1  <<<<<<<<<< */
void ex6_1 (void)
{ int ny = 1100, nl;
  static const char *ctit = "Instruction Alphabet",
  *ct1 = 
   "{m2}C{m4}(x) = {m3p}v{m4}e{e}-t{r}t{e}x-1{r}dt{gdh0.4f-1}0{u1.4m3f3}l",
  *ct2 =
   "lim{gdhc}x{m3cd1.1}a{c}l{rm} (1 + {puh} 1 {rvgd0.5h} x {r}){u1.2h}x{r} = e",
  *ct0 = 
   "Character{h0.5} height{rz-30}  incli{z30}nation {zw0.5} ratio {wk} fixed width",
  *ct3 = "Underscoring{l}    {p}twice{j}    vectors {pa8v}";

  g.setpag ("da4l");
  g.disini ();
  g.pagera ();
  g.hwfont ();
  g.color  ("cyan");

  g.height (50);
  nl = g.nlmess (ctit);
  g.messag (ctit, (2970 - nl) / 2, 250);

  g.height (36);
  g.messag ("1.)", 300, 450);
  g.messag (ct0, 500, 450);

  g.height (50);
  g.smxalf ("inst", "{", "}", 1);
  g.messag (ct0, 500, 550);

  g.reset  ("smxa");
  g.height (36);
  g.messag ("2.)", 300, 750);
  g.messag (ct3, 500, 750);

  g.height (50);
  g.smxalf ("inst", "{", "}", 1);
  g.messag (ct3, 500, 850);

  g.reset  ("smxalf");
  g.height (36);
  g.messag ("3.)", 300, ny);
  g.messag (ct1, 500, ny);

  g.smxalf ("inst", "{", "}", 1);
  g.height (80);
  g.messag (ct1, 900, ny + 150);

  g.height (36);
  g.reset  ("smxa");
  g.messag ("4.)", 300, ny + 450);
  g.messag (ct2, 500, ny + 450);

  g.height (80);
  g.smxalf ("inst", "{", "}", 1);
  g.messag (ct2, 900, ny + 600);
  g.disfin ();
}

/* >>>>>>>>>> EX10_1 <<<<<<<<<< */
void ex10_1 (void)
{ int nya = 2700, i;
  static const char *ctit = "Bar Graphs (BARS)";
  char cbuf[25];

  static double x[9]  = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0},
                y[9]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                y1[9] = {1.0, 1.5, 2.5, 1.3, 2.0, 1.2, 0.7, 1.4, 1.1},
                y2[9] = {2.0, 2.7, 3.5, 2.1, 3.2, 1.9, 2.0, 2.3, 1.8},
                y3[9] = {4.0, 3.5, 4.5, 3.7, 4.0, 2.9, 3.0, 3.2, 2.6};

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();
  g.ticks  (1, "x");
  g.intax  ();;
  g.axslen (1600, 700);
  g.titlin (ctit, 3);

  g.legini (cbuf, 3, 8);
  g.leglin (cbuf, "FIRST", 1);
  g.leglin (cbuf, "SECOND", 2);
  g.leglin (cbuf, "THIRD", 3);
  g.legtit (" ");
  g.shdpat (5L);
  for (i = 1; i <= 3; i++)
  { if (i > 1) g.labels ("none", "x");
    g.axspos (300, nya - (i - 1) * 800);
    g.graf   (0.0, 10.0, 0.0, 1.0, 0.0, 5.0, 0.0, 1.0);

    if (i == 1)
    { g.bargrp (3, 0.15);
      g.color  ("red");
      g.bars   (x, y, y1, 9);
      g.color  ("green");
      g.bars   (x, y, y2, 9);
      g.color  ("blue");
      g.bars   (x, y, y3, 9);
      g.color  ("fore");
      g.reset  ("bargrp");
    }
    else if (i == 2)
    { g.height (30);
      g.labels ("delta", "bars");
      g.labpos ("center", "bars");
      g.color  ("red");
      g.bars   (x, y, y1, 9);
      g.color  ("green");
      g.bars   (x, y1, y2, 9);
      g.color  ("blue");
      g.bars   (x, y2, y3, 9);
      g.color  ("fore");
      g.reset  ("height"); 
    }
    else if (i == 3)
    { g.labels ("second", "bars");
      g.labpos ("outside", "bars");
      g.color  ("red");
      g.bars   (x, y, y1, 9);
      g.color  ("fore");
    }

    if (i != 3) g.legend (cbuf, 7);

    if (i == 3)
    { g.height (50);
      g.title  ();
    }

    g.endgrf ();
  }
  g.disfin ();
}

/* >>>>>>>>>> EX10_2 <<<<<<<<<< */
void ex10_2 (void)
{ int nya = 2800, i;
  static const char *ctit = "Pie Charts (PIEGRF)";
  char cbuf[41];
  static double xdray[5] = {1.0, 2.5, 2.0, 2.7, 1.8};

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();
  g.axslen (1600, 1000);
  g.titlin (ctit, 2);
  g.chnpie ("both");

  g.legini (cbuf, 5, 8);
  g.leglin (cbuf, "FIRST", 1);
  g.leglin (cbuf, "SECOND", 2);
  g.leglin (cbuf, "THIRD", 3);
  g.leglin (cbuf, "FOURTH", 4);
  g.leglin (cbuf, "FIFTH", 5);

  g.patcyc (1, 7L);
  g.patcyc (2, 4L);
  g.patcyc (3, 13L);
  g.patcyc (4, 3L);
  g.patcyc (5, 5L);

  for (i = 0; i < 2; i++)
  { g.axspos (250,nya - i * 1200);
    if (i == 1)
    { g.labels ("data", "pie");
      g.labpos ("external", "pie");
    }
    g.piegrf (cbuf, 1, xdray, 5);

    if (i == 1)
    { g.height (50);
      g.title  ();
    }
    g.endgrf ();
  }
  g.disfin ();
}

/* >>>>>>>>>> EX10_3 <<<<<<<<<< */
void ex10_3 (void)
{ char cbuf[80];
  double x[5]  = {2., 4., 6., 8., 10.},
         y1[5] = {0., 0., 0., 0., 0.},
         y2[5] = {3.2, 1.5, 2.0, 1.0, 3.0};

  int ic1ray[5]  = {50, 150, 100, 200, 175},
      ic2ray[5]  = {50, 150, 100, 200, 175};

  g.setpag ("da4p");
  g.disini ();
  g.pagera ();
  g.hwfont ();

  g.titlin ("3-D Bar Graph / 3-D Pie Chart", 2);
  g.htitle (40);

  g.shdpat (16L);
  g.axslen (1500, 1000);
  g.axspos (300, 1400);

  g.barwth (0.5);
  g.bartyp ("3dvert");
  g.labels ("second", "bars");
  g.labpos ("outside", "bars");
  g.graf   (0., 12., 0., 2., 0., 5., 0., 1.);
  g.title  ();
  g.color  ("red");
  g.bars   (x, y1, y2, 5);
  g.endgrf ();

  g.shdpat (16L);
  g.labels ("data", "pie");
  g.chnpie ("none");
  g.pieclr (ic1ray, ic2ray, 5);
  g.pietyp ("3d");
  g.axspos (300, 2700);
  g.piegrf (cbuf, 0, y2, 5);       
  g.disfin ();
}

/* >>>>>>>>>> EX11_1 <<<<<<<<<< */
void ex11_1 (void)
{ int n = 50, i, j;
  double   fpi = 3.1415927 / 180., step, x, y;
  char   *cdev;

  step = 360. / (n - 1);
  for (i = 0; i < n; i++)
  { x = i * step;
    for (j = 0; j < n; j++)
    { y = j * step;
      zmat[i][j] = 2 * sin (x * fpi) * sin (y * fpi);
    }
  }

  g.disini ();
  g.pagera ();
  cdev = g.getmfl ();
  g.hwfont ();

  g.titlin ("3-D Colour Plot of the Function", 2);
  g.titlin ("F(X,Y) = 2 * SIN(X) * SIN(Y)", 4);

  g.name   ("X-axis", "x");
  g.name   ("Y-axis", "y");
  g.name   ("Z-axis", "z");

  g.intax  ();
  g.autres (n, n);
  g.axspos (300, 1850);
  g.ax3len (2200, 1400, 1400);

  g.graf3  (0.0, 360.0, 0.0, 90.0, 0.0, 360.0, 0.0, 90.0,
            -2.0, 2.0, -2.0, 1.0);
  g.crvmat((double *) zmat, n, n, 1, 1);

  g.height (50);
  if (strcmp (cdev, "POST") == 0)
        g.psfont ("palatino-bolditalic");
  g.title  ();
  g.disfin ();
}

/* >>>>>>>>>> EXA_13 <<<<<<<<<< */
void exa_13()
{ int n = 7, i;
  static const char
      *clr[7] = {"fore", "red", "yello", "cyan", "oran", "green", "blue"},
     *cont[7] = {"anta", "afri", "sout" , "aust", "nort", "eura", "lake"};

  g.disini ();
  g.hwfont ();
  g.setvlt ("small");
  g.pagera ();

  g.projct ("hammer");
  g.noclip ();
  g.grafmp (-180.0, 180.0, -180.0, 30.0, -90.0, 90.0, -90.0, 30.0);

  g.shdpat (16L);
  for (i = 0; i < n; i++)
  { g.color  (clr[i]); 
    g.shdmap (cont[i]);                               
    g.sendbf ();
  }

  g.color  ("fore");
  g.gridmp (1, 1);
  g.disfin ();
}

/* >>>>>>>>>> EXA_14 <<<<<<<<<< */
void exa_14 (void)
{ int ncol = 0, ny, i, j, nx, nb = 100, nh = 80, nl, ip, id_lis, ilis;
  double xcol;
  char   *cvlt;
  static const char clis[] =
      {"SMALL|RAINBOW|SPECTRUM|TEMPERATURE|GREY|RRAIN|RGREY|VGA"};

  g.swgpop ("NOQUIT");
  g.swgpop ("NOHELP");

  ip = g.wgini ("vert");
  ilis = 2;
  g.wglab (ip, "Colour Table:");
  id_lis = g.wglis (ip, clis, ilis);
  g.wgok (ip);
  g.wgfin ();

  ilis = g.gwglis (id_lis);
  cvlt = g.itmstr (clis, ilis);

  g.disini ();
  g.setvlt (cvlt);
  g.pagera ();

  g.height (30);
  for (i = 1; i <= 13; i++)
  { ny = i * 150;
    nx = -50;
    for (j = 1; j <= 20; j++)
    { nx += 145;
      if (ncol <= 255)
      { g.point  (nx, ny, nb, nh, ncol);
        xcol = (double) ncol;
        nl = g.nlnumb (xcol, -1);
        g.color ("fore");  
        g.number (xcol, -1, nx - nl / 2, ny + 50);
        ncol++;
      }
    }
  }                   
  g.disfin ();
}
