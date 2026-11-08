# Font attribution

`main/font_cp437.hpp` contains a byte-indexed 8x8 CP437 bitmap table adapted from the **Oldschool CGA** font (version 2.2) by VileR. The upstream glyph set is distributed under the Creative Commons Attribution-ShareAlike 4.0 International license (CC BY-SA 4.0).

Source used to generate the table: [susam/pcface oldschool-cga-8x8](https://github.com/susam/pcface/tree/main/out/oldschool-cga-8x8). Font project documentation and license details: [susam/pcface README](https://github.com/susam/pcface#license) and [Oldschool PC Fonts](https://int10h.org/oldschool-pc-fonts/).

Attribution: “Oldschool CGA font by VileR, adapted into an 8x8 CP437 glyph table.” Changes: glyph bytes are embedded in a C++ header and consumed directly by the Marauder v7 UI renderer. The font remains subject to CC BY-SA 4.0; see <https://creativecommons.org/licenses/by-sa/4.0/>.
