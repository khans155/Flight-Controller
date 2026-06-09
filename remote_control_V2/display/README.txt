RemoteDisplay_NewGUI_compilefix

This version fixes the PicoPixel/LVGL custom font compile issue.

The important change is that fonts.c no longer includes all three font .c files.
Each font is compiled through its own root-level wrapper file:

  font_14px_compile.c
  font_32px_compile.c
  font_22px_compile.c

Why:
LVGL font exports use repeated internal static names such as glyph_bitmap,
glyph_dsc, cmaps, kern_pairs, and font_dsc. These names are safe when each
font is compiled separately, but they collide if several font .c files are
included into the same fonts.c file.

Keep the fonts/ folder exactly as-is. Do not manually include the individual
font .c files in fonts.c.

You still need your i2c_display.h file in this sketch folder.
