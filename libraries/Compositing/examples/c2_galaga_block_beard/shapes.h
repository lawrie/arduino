/* f32c Galaga
   AUTHOR=EMARD
   LICENSE=BSD
   Artwork, shapes
*/
#include <stdlib.h>
#include "shape.h"

// ascii-art of the shapes
const struct charcolors std_colors[] =
{ //               RRGGBB
  {'O', RGB2PIXEL(0xFF7F00)}, // orange
  {'R', RGB2PIXEL(0xFF0000)}, // red
  {'Y', RGB2PIXEL(0xFFFF00)}, // yellow
  {'C', RGB2PIXEL(0x00FFFF)}, // cyan
  {'G', RGB2PIXEL(0x00FF00)}, // green
  {'B', RGB2PIXEL(0x0000FF)}, // blue
  {'V', RGB2PIXEL(0xFF00FF)}, // violett
  {'W', RGB2PIXEL(0xFFFFFF)}, // white
  {'.', RGB2PIXEL(0x000040)}, // blueish "beard"
  {' ', RGB2PIXEL(0)}, // transparent
  {0, 0}
};


const char *shape_alien1u[] =
{
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "WWWWBBBBWWWW",
  "WWWWBBBBWWWW",
  "WWWWBBBBWWWW",
  "WWWWBBBBWWWW",
  "............",
  "............",
  "............",
  "............",
  NULL
};

const char *shape_alien1d[] =
{
  "WWWWBBBBWWWW",
  "WWWWBBBBWWWW",
  "WWWWBBBBWWWW",
  "WWWWBBBBWWWW",
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "    ....",
  "    ....",
  "    ....",
  "    ....",
  NULL
};

const char *shape_alien1l[] =
{
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "WWWWBBBB....",
  "WWWWBBBB....",
  "WWWWBBBB....",
  "WWWWBBBB....",
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "....WWWW....",
  "........",
  "........",
  "........",
  "........",
  NULL
};

const char *shape_alien1r[] =
{
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "BBBBWWWW",
  "BBBBWWWW",
  "BBBBWWWW",
  "BBBBWWWW",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "........",
  "........",
  "........",
  "........",
  NULL
};

const char *shape_alien2u[] =
{
  "....CCCC....",
  "....CCCC....",
  "....CCCC....",
  "....CCCC....",
  "YYYYRRRRYYYY",
  "YYYYRRRRYYYY",
  "YYYYRRRRYYYY",
  "YYYYRRRRYYYY",
  "............",
  "............",
  "............",
  "............",
  NULL
};

const char *shape_alien2d[] =
{
  "YYYYRRRRYYYY",
  "YYYYRRRRYYYY",
  "YYYYRRRRYYYY",
  "YYYYRRRRYYYY",
  "....CCCC....",
  "....CCCC....",
  "....CCCC....",
  "....CCCC....",
  "............",
  "............",
  "............",
  "............",
  NULL
};

const char *shape_alien2l[] =
{
  "....YYYY....",
  "....YYYY....",
  "....YYYY....",
  "....YYYY....",
  "CCCCRRRR....",
  "CCCCRRRR....",
  "CCCCRRRR....",
  "CCCCRRRR....",
  "....YYYY....",
  "....YYYY....",
  "....YYYY....",
  "....YYYY....",
  "........",
  "........",
  "........",
  "........",

};

const char *shape_alien2r[] =
{
  "YYYY....",
  "YYYY....",
  "YYYY....",
  "YYYY....",
  "RRRRCCCC....",
  "RRRRCCCC....",
  "RRRRCCCC....",
  "RRRRCCCC....",
  "YYYY....",
  "YYYY....",
  "YYYY....",
  "YYYY....",
  "........",
  "........",
  "........",
  "........",
  NULL
};

const char *shape_alien3u[] =
{
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "....VVVV....VVVV....",
  "....VVVV....VVVV....",
  "....VVVV....VVVV....",
  "....VVVV....VVVV....",
  "WWWW....    ....WWWW",
  "WWWW....    ....WWWW",
  "WWWW....    ....WWWW",
  "WWWW....    ....WWWW",
  "........    ........",
  "........    ........",
  NULL
};

const char *shape_alien3d[] =
{
  "WWWW....    ....WWWW",
  "WWWW....    ....WWWW",
  "WWWW....    ....WWWW",
  "WWWW....    ....WWWW",
  "....VVVV....VVVV....",
  "....VVVV....VVVV....",
  "....VVVV....VVVV....",
  "....VVVV....VVVV....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "        ....",
  "        ....",
  NULL
};

const char *shape_alien3l[] =
{
  "    ....WWWW",
  "    ....WWWW",
  "    ....WWWW",
  "    ....WWWW",
  "....VVVV....",
  "....VVVV....",
  "....VVVV",
  "....VVVV",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "....VVVV",
  "....VVVV",
  "....VVVV",
  "....VVVV",
  "    ....WWWW",
  "    ....WWWW",
  "    ....WWWW",
  "    ....WWWW",
  "        ....",
  "        ....",
  NULL
};

const char *shape_alien3r[] =
{
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "....VVVV....",
  "....VVVV....",
  "....VVVV....",
  "....VVVV....",
  "    ....WWWW",
  "    ....WWWW",
  "    ....WWWW",
  "    ....WWWW",
  "....VVVV....",
  "....VVVV....",
  "....VVVV....",
  "....VVVV....",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "WWWW....",
  "....",
  "....",
  NULL
};

const char *shape_alien4u[] =
{
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "....OOOOWWWWOOOO....",
  "....OOOOWWWWOOOO....",
  "....OOOOWWWWOOOO....",
  "....OOOOWWWWOOOO....",
  "VVVV............VVVV",
  "VVVV............VVVV",
  "VVVV............VVVV",
  "VVVV............VVVV",
  "....            ....",
  "....            ....",
  NULL
};

const char *shape_alien4d[] =
{
  "VVVV....    ....VVVV",
  "VVVV....    ....VVVV",
  "VVVV....    ....VVVV",
  "VVVV....    ....VVVV",
  "....OOOOWWWWOOOO....",
  "....OOOOWWWWOOOO....",
  "....OOOOWWWWOOOO....",
  "....OOOOWWWWOOOO....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "        ....",
  "        ....",
  NULL
};

const char *shape_alien4l[] =
{
  "    ....VVVV",
  "    ....VVVV",
  "    ....VVVV",
  "    ....VVVV",
  "....OOOO....",
  "....OOOO....",
  "....OOOO",
  "....OOOO",
  "WWWWWWWW",
  "WWWWWWWW",
  "WWWWWWWW",
  "WWWWWWWW",
  "....OOOO",
  "....OOOO",
  "....OOOO",
  "....OOOO",
  "    ....VVVV",
  "    ....VVVV",
  "    ....VVVV",
  "    ....VVVV",
  "        ....",
  "        ....",
  NULL
};

const char *shape_alien4r[] =
{
  "VVVV....",
  "VVVV....",
  "VVVV....",
  "VVVV....",
  "....OOOO....",
  "....OOOO....",
  "....OOOO....",
  "....OOOO....",
  "....WWWWWWWW",
  "....WWWWWWWW",
  "....WWWWWWWW",
  "....WWWWWWWW",
  "....OOOO....",
  "....OOOO....",
  "....OOOO....",
  "....OOOO....",
  "VVVV....",
  "VVVV....",
  "VVVV....",
  "VVVV....",
  "....",
  "....",
  NULL
};

const char *shape_alien5u[] =
{
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "....OOOOYYYYOOOO",
  "....OOOOYYYYOOOO",
  "....OOOOYYYYOOOO",
  "....OOOOYYYYOOOO",
  "GGGG....WWWW....GGGG",
  "GGGG....WWWW....GGGG",
  "GGGG....WWWW....GGGG",
  "GGGG....WWWW....GGGG",
  "........WWWW........",
  "........WWWW........",
  "........WWWW........",
  "........WWWW........",
  "....WWWW....WWWW....",
  "....WWWW....WWWW....",
  "....WWWW....WWWW....",
  "....WWWW....WWWW....",
  "    ....    ....",
  "    ....    ....",
  NULL
};

const char *shape_alien5d[] =
{
  "....WWWW....WWWW....",
  "....WWWW....WWWW....",
  "....WWWW....WWWW....",
  "....WWWW....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "GGGG....WWWW....GGGG",
  "GGGG....WWWW....GGGG",
  "GGGG....WWWW....GGGG",
  "GGGG....WWWW....GGGG",
  "....OOOOYYYYOOOO....",
  "....OOOOYYYYOOOO....",
  "....OOOOYYYYOOOO....",
  "....OOOOYYYYOOOO....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "    ....WWWW....",
  "        ....",
  "        ....",
  NULL
};

const char *shape_alien5l[] =
{
  "    ....GGGG",
  "    ....GGGG",
  "    ....GGGG",
  "    ....GGGG",
  "....OOOO....    WWWW",
  "....OOOO....    WWWW",
  "....OOOO....    WWWW",
  "....OOOO....    WWWW",
  "WWWWWWWWWWWWWWWW....",
  "WWWWWWWWWWWWWWWW....",
  "WWWWWWWWWWWWWWWW....",
  "WWWWWWWWWWWWWWWW....",
  "....OOOO........WWWW",
  "....OOOO........WWWW",
  "....OOOO........WWWW",
  "....OOOO........WWWW",
  "    ....GGGG........",
  "    ....GGGG........",
  "    ....GGGG........",
  "    ....GGGG........",
  "        ....",
  "        ....",
  NULL
};

const char *shape_alien5r[] =
{
  "    ....GGGG....",
  "    ....GGGG....",
  "    ....GGGG....",
  "    ....GGGG....",
  "WWWW........OOOO",
  "WWWW........OOOO",
  "WWWW........OOOO",
  "WWWW........OOOO",
  "....WWWWWWWWWWWWWWWW",
  "....WWWWWWWWWWWWWWWW",
  "....WWWWWWWWWWWWWWWW",
  "....WWWWWWWWWWWWWWWW",
  "WWWW........OOOO....",
  "WWWW........OOOO....",
  "WWWW........OOOO....",
  "WWWW........OOOO....",
  "........GGGG....",
  "........GGGG....",
  "........GGGG....",
  "........GGGG....",
  "        ....",
  "        ....",
  NULL
};

const char *shape_ship1u[] =
{
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "WWWW....WWWW",
  "WWWW....WWWW",
  "WWWW    WWWW",
  "WWWW    WWWW",
  "....    ....",
  "....    ....",
  NULL
};

const char *shape_ship1d[] =
{
  "WWWW    WWWW",
  "WWWW    WWWW",
  "WWWW    WWWW",
  "WWWW    WWWW",
  "....WWWW....",
  "....WWWW....",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    WWWW",
  "    ....",
  "    ....",
  NULL
};

const char *shape_ship1l[] =
{
  "        WWWW",
  "        WWWW",
  "        WWWW",
  "        WWWW",
  "WWWWWWWW....",
  "WWWWWWWW....",
  "WWWWWWWW",
  "WWWWWWWW",
  "........WWWW",
  "........WWWW",
  "        WWWW",
  "        WWWW",
  "        ....",
  "        ....",
  NULL
};

const char *shape_ship1r[] =
{
  "WWWW",
  "WWWW",
  "WWWW",
  "WWWW",
  "....WWWWWWWW",
  "....WWWWWWWW",
  "    WWWWWWWW",
  "    WWWWWWWW",
  "WWWW........",
  "WWWW........",
  "WWWW",
  "WWWW",
  "....",
  "....",
  NULL
};

const char *shape_ship2u[] =
{
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "    WWWW    WWWW",
  "WWWW....WWWW....WWWW",
  "WWWW....WWWW....WWWW",
  "WWWW    WWWW    WWWW",
  "WWWW    WWWW    WWWW",
  "....    ....    ....",
  "....    ....    ....",
  NULL
};

const char *shape_alien_suction1[] =
{
  "GGGG",
  "GGGG",
  "GGGG",
  "GGGG",
  "....",
  "....",
  NULL
};

const char *shape_alien_suction3[] =
{
  "GGGGGGGGGGGG",
  "GGGGGGGGGGGG",
  "GGGGGGGGGGGG",
  "GGGGGGGGGGGG",
  "............",
  "............",
  NULL
};

const char *shape_alien_suction5[] =
{
  "GGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGG",
  "....................",
  "....................",
  NULL
};

const char *shape_alien_suction7[] =
{
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "............................",
  "............................",
  NULL
};

const char *shape_alien_suction9[] =
{
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "....................................",
  "....................................",
  NULL
};

const char *shape_alien_suction11[] =
{
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG",
  "............................................",
  "............................................",
  NULL
};

const char *shape_bomb[] =
{
  "WWWW",
  "WWWW",
  "WWWW",
  "WWWW",
  "....",
  "....",
  NULL
};

const char *shape_missile0[] =
{
  " WW ",
  " .WW",
  " WW.",
  "WW..",
  ".WW ",
  "..WW",
  " WW.",
  "WW..",
  ".WW ",
  "..WW",
  ".WW.",
  "....",
  "....",
  "....",
  "....",
  NULL
};

const char *shape_missile1[] =
{
  "  WW",
  " WW.",
  "WW..",
  ".WW ",
  " .WW",
  " WW.",
  "WW..",
  ".WW ",
  "..WW",
  " WW.",
  "WW..",
  "....",
  "....",
  "....",
  "....",
  NULL
};

const char *shape_missile2[] =
{
  " WW ",
  "WW. ",
  ".WW ",
  "..WW",
  " WW.",
  "WW..",
  ".WW ",
  "..WW",
  " WW.",
  "WW..",
  ".WW ",
  "....",
  "....",
  "....",
  "....",
  NULL
};

const char *shape_missile3[] =
{
  "WW  ",
  ".WW ",
  "..WW",
  " WW.",
  "WW..",
  ".WW ",
  "..WW",
  " WW.",
  "WW..",
  ".WW ",
  "..WW",
  "....",
  "....",
  "....",
  "....",
  NULL
};

const char *shape_block_red[] =
{
  "RRRR",
  "RRRR",
  "RRRR",
  "RRRR",
  "....",
  "....",
  NULL
};

const char *shape_block_yellow[] =
{
  "YYYY",
  "YYYY",
  "YYYY",
  "YYYY",
  "....",
  "....",
  NULL
};

const char *shape_block_orange[] =
{
  "OOOO",
  "OOOO",
  "OOOO",
  "OOOO",
  "....",
  "....",
  NULL
};

const char *shape_block_green[] =
{
  "GGGG",
  "GGGG",
  "GGGG",
  "GGGG",
  "....",
  "....",
  NULL
};

const char *shape_block_cyan[] =
{
  "CCCC",
  "CCCC",
  "CCCC",
  "CCCC",
  "....",
  "....",
  NULL
};

const char *shape_block_blue[] =
{
  "BBBB",
  "BBBB",
  "BBBB",
  "BBBB",
  "....",
  "....",
  NULL
};

const char *shape_block_violett[] =
{
  "VVVV",
  "VVVV",
  "VVVV",
  "VVVV",
  "....",
  "....",
  NULL
};

const char *shape_block_white[] =
{
  "WWWW",
  "WWWW",
  "WWWW",
  "WWWW",
  "....",
  "....",
  NULL
};

const char *shape_firebally0[] =
{
  "         WYWWWW                  Y                              ",
  "       WWWWWWWWWWY      Y        .      Y     YWWWWWY      Y    ",
  "     WWWWWWWWWWWWWWW    .    YYWWWWY    .   WWWWWWWWWWY    .    ",
  "    WWYWWWWWWWWWWWWWWY  .  WWWWWWWWWWY  WWYWWWWWWWWWWWWWY  .    ",
  "   WWWWWWWWWWWWWWWWWWWW   YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW      ",
  "   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY     ",
  "  WYWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "  WWWYWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY   ",
  "   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY   ",
  "  WWWYWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "  .YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "  .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY   ",
  "   .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.   ",
  "   ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.   ",
  "    ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY.    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW..    ",
  "     .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY..     ",
  "     .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY..    Y ",
  "  Y   .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.   Y . ",
  "  .   ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    . . ",
  "  .    ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY    .   ",
  "        ..YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.        ",
  "         ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.        ",
  " w    Y   ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.     Y   ",
  " .    .    ..YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY.     .   ",
  " .    .W    ...WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  Y  .   ",
  "   Y   .    W...YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  .      ",
  "   .   .    .  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY .      ",
  "   .      Y . W.WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW      ",
  "          .   .WWYWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY     ",
  "          .   YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "             YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY   ",
  "           YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "          WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY  ",
  " Y       WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "        WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "       WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY ",
  "      YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY",
  "    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "   YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY",
  "   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  " YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY",
  " WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWYWWWWWWWWWWWWWWWWWWWWWWWWWWWW.",
  "YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW......YWWWWWWWWWWWWWWWWWWWWWWY.",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY.Y....WWWWWWWWWWWWWWWWWWWWWW. ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ..   .WWWWWWWWWWWWWWWWWWWWWY. ",
  "YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ..Y  ..WWWWWWWWWWWWWWWWWWWWW  ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY   .   ..YWWWWWWWWWWWWWWWWWWW  ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.   .    ..WWWWWWWWWWWWWWWWWWY  ",
  "YWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWY.  Y      ..WWWWWWWWWWWWWWWWY.  ",
  ".WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.   .       .YWWWWWWWWWWWWWWWW.  ",
  ".WWWWWWWWWWWWWWWWWWWWWWWWWWWWW..   .        .WWWWWWWWWWWWWWY.   ",
  " .YWWWWWWWWWWWWWWWWWWWWWWWWWY..       Y     ..YWWWWWWWWWWWW..   ",
  " ..WWWWWWWWWWWWWWWWWWWWWYWY...        .    Y . WWWWWWWWWWW..  Y ",
  "Y ..YWWWWWWWWWWWWWWWY........         .    .   ..YWWWWWY...     ",
  ".  ...YWWWWWYWWWWYY........     Y          .   .........        ",
  ".   ...............             .                .......",
  "      .............             .                .......",
  NULL
};

const char *shape_firebally1[] =
{
  "      O                            O      WWWWWWW        ",
  " O         WWWWWWW        O             WWWWWWWWWWW   O  ",
  "         WWWWWWWWWWW           O       WWWWWWWWWWWWW     ",
  "        WWWWWWWWWWWWW   O             WWWWWWWWWWWWWWW    ",
  "      WWWWWWWWWWWWWWWW       WWWWWWW WWWWWWWWWWWWWWWWW   ",
  "  O  WWWWWWWWWWWWWWWWWW    WWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "    WWWWWWWWWWWWWWWWWWW   WWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "   WWWWWWWWWWWWWWWWWWWWW WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "   .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.",
  "   ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.",
  "    ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW. ",
  "      ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.. ",
  "   O  ......WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.. O",
  "   .    ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW...  .",
  "   . O   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW....   .",
  "     .  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.. O    ",
  "     . WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.   .    ",
  "       WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   .    ",
  "      WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW     O  ",
  "      WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   .  ",
  " O    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  .  ",
  " .    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  " .    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "      WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "      WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "   W  .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "   .  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "   .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  " WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  " WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWO....WWWWWWWWWWWWWWWWWWW",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW..O..WWWWWWWWWWWWWWWWWWW",
  ".WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  .  .WWWWWWWWWWWWWWWWW.",
  ".WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  .  .WWWWWWWWWWWWWWWWW.",
  " .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.      .WWWWWWWWWWWWWWW. ",
  " ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWW.    O ..WWWWWWWWWWWWW.. ",
  "  ..WWWWWWWWWWW.WWWWWWWWWWWWWWW.     .   .WWWWWWWWWWW..  ",
  "    ..WWWWWWW.. .WWWWWWWWWWWWW..     .   ...WWWWWWW... O ",
  "  O   .........O..WWWWWWWWWWW..   O         .......    . ",
  "  .   .O.....  .  ..WWWWWWW...    .         ......O    . ",
  "  .    ......  .  ...........     .               .      ",
  "       .            .......                       .      ",
  NULL
};

const char *shape_firebally2[] =
{
  "                                       R     OOOOOOOOO        ",
  "     R                                 .   OOOOOOOOOOOOO   R  ",
  "     .           R                     . OOOOOOOOOOOOOOOO  .  ",
  "     .           .               R     OOOOOOOOOOOOOOOOOOO .  ",
  "      OOOOOOO    .   OOOOOOO     .    OOOOOOOOOOOOOOOOOOOO    ",
  "    OOOOOOOOOOO    OOOOOOOOOOOOO .   OOOOOOOOOOOOOOOOOOOOOO   ",
  "   OOOOOOOOOOOOO  OOOOOOOOOOOOOOOO  OOOOOOOOOOOOOOOOOOOOOOO   ",
  "  OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO OOOOOOOOOOOOOOOOOOOOOOO   ",
  " OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   ",
  " OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   ",
  "OOOOOOOOOOO.OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   ",
  "OOOOOOOOOO...OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   ",
  "OOOOOOOOOO. ...OOOOOOOOOOOOOOOOOOOOOOOOOOOOO....OOOOOOOOOO.   ",
  "OOOOOOOOOOOO...OOOOOOOOOOOOOOOOOOOOOOOOOO......OOOOOOOOOOO.   ",
  "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO....OOOOOOOOOOO R  ",
  "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO .  ",
  "OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO .  ",
  ".OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO    ",
  ".OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   R",
  " .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   .",
  " ..OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.   .",
  "  ..OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.    ",
  "   ...OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   R  ",
  "  R   ...OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.OOOOOOOOOOOOOOOO .  ",
  "  .   ...OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO....OOOOOOOOOOOOOOOO.  ",
  "  .   R  OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.....OOOOOOOOOOOOOOOO  ",
  "      .  OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.OOOOOOOOOOOOOOOOOOO ",
  "      .  .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO ",
  "   R     .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "   .      .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "   .  R   .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "      .    .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "      .    ..OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "R          OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  ".         OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  ".        OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.",
  "    R   OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.",
  "    .   OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO. ",
  "    .  OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO. ",
  "     R OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO ",
  "     . OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "     . OOOOOOOOOOOOOO...OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "       OOOOOOOOOOOOO.....OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "       OOOOOOOOOOOOOO...OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "  R    OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO..OOOOOOOO",
  "  .    .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.....OOOOOO",
  "  .    .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO..OOOOOO",
  "        OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "    R   OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO",
  "    .   OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO.OOOOOOOOOOOOOOOOOOOOO",
  "    .   OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO...OOOOOOOOOOOOOOOOOOO",
  "  R     .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   ...OOOOOOOOOOOOOOO.",
  "  .     .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO   R  .OOOOOOOOOOOOO..",
  "  .      .OOOOOOOOOOOOOOOOOOOOOOOOOOOOO.   .  ..OOOOOOOOOOO.. ",
  "      R  ..OOOOOOOOOOOOOOOOOOOOOOOOOOOO.  R.   ...OOOOOOO.. R ",
  "      .    .OOOOOOOOOOOOOOOOOOOOOOOOOO.   .     ........... . ",
  "      .    ...OOOOOOO...OOOOOOOOOOOOO..   .    R   R        . ",
  "    R      R ............OOOOOOOOOOO..    R    .   .        R ",
  "    .      .  .......   ...OOOOOOO...     .    .   .        . ",
  "    .      .             ...........      .                 .",
  "                           .......",
  NULL
};

const char *shape_firebally3[] =
{
  "R               R       RRRRRRR   R    R ",
  ".     RRRRRRR   .     RRRRRRRRRRR .    . ",
  ".   RRRRRRRRRRR . R  RRRRRRRRRRRRR.    . ",
  "   RRRRRRRRRRRRR  . RRRRRRRRRRRRRRR    R ",
  "  RRRRRRRRRRRRRRR .RRRRRRRRRRRRRRRRR   . ",
  " RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR   . ",
  " RRRRRRRRRRRRRRRRRRRRRRR..RRRRRRRRRRR    ",
  "RRRRRRRRRRRRRRRRRRRRRRR.....RRRRRRRRR    ",
  "RRRRRRRRRRRRRRRRRRRRRRRR..RRRRRRRRRRR R  ",
  "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR .  ",
  "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR .  ",
  "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR    ",
  "RRRRRR...RRRRRRRRRRRRRRRRRRRRRRRRRRRR    ",
  "RRRRRRR...RRRRRRRRRRRRRRRRRRRRRRRRRR.  R ",
  ".RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR.  . ",
  ".RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR. R . ",
  " .RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR. .   ",
  " ..RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR  .  R",
  "  . RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR R   .",
  " R  .RRRRRRRRRRRRRRRRRRRRRRRRRRRRRR .   .",
  " .  .RRRRRRRRRRRRRRRRRRRRRRRRRRRRR. .    ",
  " .  RRRRRRRRRRRRRRRRRRRRRRRRRRRRRR.   R  ",
  "    RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR   .  ",
  "R   RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR .  ",
  ".   RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR   ",
  ".   RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR  ",
  "    RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR ",
  "   RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR ",
  "   RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRR...RRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRRR..RRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR",
  "  RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR.",
  "  .RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR.",
  "  .RRRRRRRRRRRRRRRRRRR.RRRRRRRRRRRRRRRR. ",
  "    RRRRRRRRRRRRRRRRR....RRRRRRRRRRRRR..R",
  " R   RRRRRRRRRRRRRRR......RRRRRRRRRRR.. .",
  " .    RRRRRRRRRRRR...  R  ..RRRRRRR..   .",
  " .  R ..RRRRRRR....R   .   .........R   R",
  "    . ..............   .    ....... .   .",
  "    .   .......    .                .   .",
  NULL
};

const char *shape_fireballb0[] =
{
  "                          WWWWWWW                               ",
  "                        WWWWWWWWWWW                             ",
  "                       WWWWWWWWWWWWW                            ",
  "                      WWWWWWWWWWWWWWW                           ",
  "                     WWWWWWWWWWWWWWWWW                          ",
  "                     WWWWWWWWWWWWWWWWW                          ",
  "                    WWWWWWWWWWWWWWWWWWW                         ",
  "                    WWWWWWWWWWWWWWWWWWW                         ",
  "                    WWWWWWWWWWWWWWWWWWW                         ",
  "                    WWWWWWWWWWWWWWWWWWW                         ",
  "                    WWWWWWWWWWWWWWWWWWW                         ",
  "                    WWWWWWWWWWWWWWWWWWWWW                       ",
  "                    WWWWWWWWWWWWWWWWWWWWWW                      ",
  "                    .WWWWWWWWWWWWWWWWWWWWWW                     ",
  "                    .WWWWWWWWWWWWWWWWWWWWWWW                    ",
  "                     WWWWWWWWWWWWWWWWWWWWWWW                    ",
  "                    WWWWWWWWWWWWWWWWWWWWWWWWW      WWWWWWW      ",
  "                    WWWWWWWWWWWWWWWWWWWWWWWWWW   WWWWWWWWWWW    ",
  "                    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW   ",
  "                    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW  ",
  "                   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  "                   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW ",
  "                  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "                 WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "                WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "      WWWWWWW   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "    WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "   WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  "  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW",
  " WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.",
  " WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW. ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.. ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW..  ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW...   ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW........    ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.......      ",
  "WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.            ",
  ".WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.             ",
  ".WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW..             ",
  " .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.              ",
  " ..WWWWWWWWWWWWW...WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW               ",
  "  ..WWWWWWWWWWW.....WWWWWWWWWWWWWWWWWWWWWWWWWWWWW               ",
  "   ...WWWWWWW...   .WWWWWWWWWWWWWWWWWWWWWWWWWWWW.               ",
  "      ........      .WWWWWWWWWWWWWWWWWWWWWWWWWWW.               ",
  "      .......       .WWWWWWWWWWWWWWWWWWWWWWWWWW.                ",
  "                     WWWWWWWWWWWWWWWWWWWWWWWWW..                ",
  "                     .WWWWWWWWWWWWWWWWWWWWWWW..                 ",
  "                     .WWWWWWWWWWWWWWWWWWWWW...                  ",
  "                      .WWWWWWWWWWWWWWWWWWWW..                   ",
  "                      ..WWWWWWWWWWWWWWWWWWW                     ",
  "                       ..WWWWWWWWWWWWWWWWWWW                    ",
  "                        .WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         .WWWWWWWWWWWWWWWWW.                    ",
  "                         .WWWWWWWWWWWWWWWWW.                    ",
  "                          .WWWWWWWWWWWWWWW.                     ",
  "                          ..WWWWWWWWWWWWW..                     ",
  "                           ..WWWWWWWWWWW..                      ",
  "                            ...WWWWWWW...                       ",
  "                             ...........",
  "                               .......",
  NULL
};

const char *shape_fireballb1[] =
{
  "                             WWWWWWW                            ",
  "                           WWWWWWWWWWW                          ",
  "                          WWWWWWWWWWWWW                         ",
  "                         WWWWWWWWWWWWWWW                        ",
  "                        WWWWWWWWWWWWWWWWW                       ",
  "                        WWWWWWWWWWWWWWWWW                       ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWW                      ",
  "                       .WWWWWWWWWWWWWWWWWW                      ",
  "                       WWWWWWWWWWWWWWWWWWWWW                    ",
  "                     WWWWWWWWWWWWWWWWWWWWWWWW                   ",
  "                    WWWWWWWWWWWWWWWWWWWWWWWWWW                  ",
  "                   WWWWWWWWWWWWWWWWWWWWWWWWWWWW                 ",
  "                  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW          ",
  "                  WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW        ",
  "                 WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW       ",
  "                 WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW      ",
  "           WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW     ",
  "         WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW     ",
  "        WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "       WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "      WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "      WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.    ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.     ",
  "     WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW..     ",
  "     .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW..      ",
  "     .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW...       ",
  "      .WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.....        ",
  "      ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW.....          ",
  "       ..WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW....             ",
  "         ..WWWWWWW..WWWWWWWWWWWWWWWWWWWWWWWWWW...               ",
  "          ...........WWWWWWWWWWWWWWWWWWWWWWWWW.                 ",
  "           .......  ..WWWWWWWWWWWWWWWWWWWWWWW.                  ",
  "                     ..WWWWWWWWWWWWWWWWWWWWW..                  ",
  "                      ...WWWWWWWWWWWWWWWWWW..                   ",
  "                       ..WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         WWWWWWWWWWWWWWWWWWW                    ",
  "                         .WWWWWWWWWWWWWWWWW.                    ",
  "                         .WWWWWWWWWWWWWWWWW.                    ",
  "                          .WWWWWWWWWWWWWWW.                     ",
  "                          ..WWWWWWWWWWWWW..                     ",
  "                           ..WWWWWWWWWWW..                      ",
  "                             ..WWWWWWW...                       ",
  "                             ...........",
  "                               .......",
  NULL
};

const char *shape_fireballb2[] =
{
  "                              CCCCCCC                           ",
  "                            CCCCCCCCCCC                         ",
  "                           CCCCCCCCCCCCC                        ",
  "                          CCCCCCCCCCCCCCC                       ",
  "                         CCCCCCCCCCCCCCCCC                      ",
  "                         CCCCCCCCCCCCCCCCC                      ",
  "                        CCCCCCCCCCCCCCCCCCC                     ",
  "                        CCCCCCCCCCCCCCCCCCCCC                   ",
  "                       CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC           ",
  "                     CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC         ",
  "                    CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC        ",
  "                   CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC       ",
  "                CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC      ",
  "              CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC      ",
  "             CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "            CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "           CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "          CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "         CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "         CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC     ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.     ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.     ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.      ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC..      ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.       ",
  "        CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC        ",
  "        .CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.        ",
  "        .CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.        ",
  "         .CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.         ",
  "         ..CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC..         ",
  "          ..CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC..          ",
  "           ...CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.           ",
  "             ...CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.            ",
  "              ..CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.            ",
  "                .CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC.             ",
  "                ..CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC..             ",
  "                 ..CCCCCCCCCCCCCCCCCCCCCCCCCCCCC..              ",
  "                  ...CCCCCCCCCCCCCCCCCCCCCCCCC...               ",
  "                   .....CCCCCCCCCCCCCCCCCCC.....                ",
  "                     ...CCCCCCCCCCCCCCCCCCC...                  ",
  "                        .CCCCCCCCCCCCCCCCC.                     ",
  "                        .CCCCCCCCCCCCCCCCC.                     ",
  "                         .CCCCCCCCCCCCCCC.                      ",
  "                         ..CCCCCCCCCCCCC..                      ",
  "                          ..CCCCCCCCCCC..                       ",
  "                            ..CCCCCCC...                        ",
  "                            ...........",
  "                              .......",
  NULL
};

const char *shape_fireballb3[] =
{
  "                               BBBBBBB                          ",
  "                             BBBBBBBBBBB                        ",
  "                            BBBBBBBBBBBBB                       ",
  "                           BBBBBBBBBBBBBBB                      ",
  "                          BBBBBBBBBBBBBBBBB                     ",
  "                          BBBBBBBBBBBBBBBBB                     ",
  "                         BBBBBBBBBBBBBBBBBBB                    ",
  "                         BBBBBBBBBBBBBBBBBBB                    ",
  "                         BBBBBBBBBBBBBBBBBBBB                   ",
  "                         BBBBBBBBBBBBBBBBBBBB                   ",
  "                        BBBBBBBBBBBBBBBBBBBBBBBBB               ",
  "                       BBBBBBBBBBBBBBBBBBBBBBBBBBBB             ",
  "                      BBBBBBBBBBBBBBBBBBBBBBBBBBBBBB            ",
  "                    BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB           ",
  "                  BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB          ",
  "                 BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB          ",
  "                BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "               BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "               BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB.         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB.         ",
  "              BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB.          ",
  "              .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB..          ",
  "              .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB..           ",
  "               .BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB...            ",
  "               ..BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB...             ",
  "                ..BBBBBBBBBBBBBBBBBBBBBBBBBBBBB..               ",
  "                 ...BBBBBBBBBBBBBBBBBBBBBBBBBB..                ",
  "                  .....BBBBBBBBBBBBBBBBBBBBBBB.                 ",
  "                    ....BBBBBBBBBBBBBBBBBBBBBB                  ",
  "                        .BBBBBBBBBBBBBBBBBBBBB                  ",
  "                        ..BBBBBBBBBBBBBBBBBBB.                  ",
  "                         ...BBBBBBBBBBBBBBBBB.                  ",
  "                          ...BBBBBBBBBBBBBBB.                   ",
  "                            ..BBBBBBBBBBBBB..                   ",
  "                             ..BBBBBBBBBBB..                    ",
  "                              ...BBBBBBB...                     ",
  "                               ...........",
  "                                 .......",
  NULL
};

enum
{
  SH_ALIEN1R = 0,  SH_ALIEN1U, SH_ALIEN1L, SH_ALIEN1D,
  SH_ALIEN2R = 4,  SH_ALIEN2U, SH_ALIEN2L, SH_ALIEN2D,
  SH_ALIEN3R = 8,  SH_ALIEN3U, SH_ALIEN3L, SH_ALIEN3D,
  SH_ALIEN4R = 12, SH_ALIEN4U, SH_ALIEN4L, SH_ALIEN4D,
  SH_ALIEN5R = 16, SH_ALIEN5U, SH_ALIEN5L, SH_ALIEN5D,
  SH_SHIP1R = 20, SH_SHIP1U, SH_SHIP1L, SH_SHIP1D, SH_SHIP2,
  SH_ALIEN_SUCTION1, SH_ALIEN_SUCTION3, SH_ALIEN_SUCTION5, SH_ALIEN_SUCTION7, SH_ALIEN_SUCTION9, SH_ALIEN_SUCTION11,
  SH_MISSILE0, SH_MISSILE1, SH_MISSILE2, SH_MISSILE3,
  SH_BLOCK_RED, SH_BLOCK_ORANGE, SH_BLOCK_YELLOW, SH_BLOCK_GREEN, SH_BLOCK_CYAN, SH_BLOCK_BLUE, SH_BLOCK_VIOLETT, SH_BLOCK_WHITE,
  SH_FIREBALLY0, SH_FIREBALLY1, SH_FIREBALLY2, SH_FIREBALLY3,
  SH_FIREBALLB0, SH_FIREBALLB1, SH_FIREBALLB2, SH_FIREBALLB3,
  SH_PLACEHOLDER,
};

const struct shape Shape[] =
{
  [SH_ALIEN1R] = { std_colors, shape_alien1r },
  [SH_ALIEN1U] = { std_colors, shape_alien1u },
  [SH_ALIEN1L] = { std_colors, shape_alien1l },
  [SH_ALIEN1D] = { std_colors, shape_alien1d },

  [SH_ALIEN2R] = { std_colors, shape_alien2r },
  [SH_ALIEN2U] = { std_colors, shape_alien2u },
  [SH_ALIEN2L] = { std_colors, shape_alien2l },
  [SH_ALIEN2D] = { std_colors, shape_alien2d },

  [SH_ALIEN3R] = { std_colors, shape_alien3r },
  [SH_ALIEN3U] = { std_colors, shape_alien3u },
  [SH_ALIEN3L] = { std_colors, shape_alien3l },
  [SH_ALIEN3D] = { std_colors, shape_alien3d },

  [SH_ALIEN4R] = { std_colors, shape_alien4r },
  [SH_ALIEN4U] = { std_colors, shape_alien4u },
  [SH_ALIEN4L] = { std_colors, shape_alien4l },
  [SH_ALIEN4D] = { std_colors, shape_alien4d },

  [SH_ALIEN5R] = { std_colors, shape_alien5r },
  [SH_ALIEN5U] = { std_colors, shape_alien5u },
  [SH_ALIEN5L] = { std_colors, shape_alien5l },
  [SH_ALIEN5D] = { std_colors, shape_alien5d },

  [SH_SHIP1R] = { std_colors, shape_ship1r },
  [SH_SHIP1U] = { std_colors, shape_ship1u },
  [SH_SHIP1L] = { std_colors, shape_ship1l },
  [SH_SHIP1D] = { std_colors, shape_ship1d },
  [SH_SHIP2]  = { std_colors, shape_ship2u },

  [SH_ALIEN_SUCTION1]  = { std_colors, shape_alien_suction1 },
  [SH_ALIEN_SUCTION3]  = { std_colors, shape_alien_suction3 },
  [SH_ALIEN_SUCTION5]  = { std_colors, shape_alien_suction5 },
  [SH_ALIEN_SUCTION7]  = { std_colors, shape_alien_suction7 },
  [SH_ALIEN_SUCTION9]  = { std_colors, shape_alien_suction9 },
  [SH_ALIEN_SUCTION11] = { std_colors, shape_alien_suction11 },

  [SH_MISSILE0] = { std_colors, shape_missile0 },
  [SH_MISSILE1] = { std_colors, shape_missile1 },
  [SH_MISSILE2] = { std_colors, shape_missile2 },
  [SH_MISSILE3] = { std_colors, shape_missile3 },

  [SH_BLOCK_RED] = { std_colors, shape_block_red },
  [SH_BLOCK_ORANGE] = { std_colors, shape_block_orange },
  [SH_BLOCK_YELLOW] = { std_colors, shape_block_yellow },
  [SH_BLOCK_GREEN] = { std_colors, shape_block_green },
  [SH_BLOCK_CYAN] = { std_colors, shape_block_cyan },
  [SH_BLOCK_BLUE] = { std_colors, shape_block_blue },
  [SH_BLOCK_VIOLETT] = { std_colors, shape_block_violett },
  [SH_BLOCK_WHITE] = { std_colors, shape_block_white },

  [SH_FIREBALLY0] = { std_colors, shape_firebally0 },
  [SH_FIREBALLY1] = { std_colors, shape_firebally1 },
  [SH_FIREBALLY2] = { std_colors, shape_firebally2 },
  [SH_FIREBALLY3] = { std_colors, shape_firebally3 },

  [SH_FIREBALLB0] = { std_colors, shape_fireballb0 },
  [SH_FIREBALLB1] = { std_colors, shape_fireballb1 },
  [SH_FIREBALLB2] = { std_colors, shape_fireballb2 },
  [SH_FIREBALLB3] = { std_colors, shape_fireballb3 },

  [SH_PLACEHOLDER] = { std_colors, shape_block_white },
  // active sprites initially have SH_PLACEHOLDER and
  // are dynamically reshaped with other shapes
};

