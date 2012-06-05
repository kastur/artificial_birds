#include <iostream>
#include <cstring>
#include <string>
#include "discpp.h"

#define NPROJ 14

using namespace std;

int id_lis, id_draw, id_but, id_lab1, id_lab2, id_quit;

string clis;
string cl1[NPROJ]  = {"Cylindrical Equidistant",
                           "Mercator",
                           "Cylindrical Equal-Area",
                           "Hammer (Elliptical)",
                           "Aitoff (Elliptical)",
                           "Winkel (Elliptical)",
                           "Sanson (Elliptical)",
                           "Conical Equidistant",
                           "Conical Equal-Area",
                           "Conical Conformal",
                           "Azimuthal Equidistant",
                           "Azimuthal Equal-Area",
                           "Azimuthal Stereographic",
                           "Azimuthal Orthgraphic"};

string cl2[NPROJ] = {"CYLI", "MERC", "EQUA", "HAMM", "AITO", "WINK",
                           "SANS", "CONI", "ALBE", "CONF", "AZIM", "LAMB",
                           "STER", "ORTH"};

void myplot (int id);

Dislin g;

int main ()
{ int i, ip, ip1, ip2;

  clis = cl1[0];
  for (i = 1; i < NPROJ; i++)
  { clis += "|";
    clis += cl1[i];
  }

  g.swgtit ("DISLIN Map Plot");
  ip = g.wgini ("hori");
  g.swgwth (-15);
  ip1 = g.wgbas (ip, "vert");
  g.swgwth (-50);
  ip2 = g.wgbas (ip, "vert");

  g.swgdrw (2100. / 2970.);
  id_lab1 = g.wglab (ip1, "Projection:");
  id_lis  = g.wglis (ip1, clis.c_str (), 1);

  id_but  = g.wgpbut (ip1, "Plot");
  g.swgcbk (id_but, myplot); 

  id_quit = g.wgquit (ip1);
  id_lab2 = g.wglab (ip2, "DISLIN Draw Widget:");
  id_draw = g.wgdraw (ip2);
  g.wgfin ();
  return 0;
}  

void myplot (int id)
{ string ctitle;
  int isel;
  double xa, xe, xorg, xstp, ya, ye, yorg, ystp;

  if (id != id_but) return;    /* Dummy statement */

  xa = -180.;
  xe = 180.;
  xorg = -180.;
  xstp = 60.;

  ya =  -90.; 
  ye =  90.;
  yorg =  -90.;
  ystp = 30.;

  isel = g.gwglis (id_lis);
  g.setxid (id_draw, "widget");
  g.metafl ("xwin");
  g.disini ();
  g.erase  ();
  g.hwfont ();

  if (isel >=4 && isel <= 7) 
    g.noclip ();
  else if (isel == 2)
  { ya = -85;
    ye = 85;
    yorg = -60;
  }
  else if (isel >= 8 && isel <= 10)
  { ya = 0;
    ye = 90;
    yorg = 0;
  }

  g.labdig (-1, "xy");
  g.name ("Longitude", "x");
  g.name ("Latitude", "y");

  g.projct (cl2[isel-1].c_str ());
  g.htitle (50);
  ctitle = cl1[isel-1];
  ctitle += " Projection";

  g.titlin (ctitle. c_str (), 3);
  g.grafmp (xa, xe, xorg, xstp, ya, ye, yorg, ystp);
  g.title  ();
  g.gridmp (1,1);
  g.color  ("green");
  g.world  ();
  g.errmod ("protocol", "off");
  g.disfin ();
}
