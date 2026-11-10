#ifndef MANGO_FULL_MAP_TREE_POSITIONS_H
#define MANGO_FULL_MAP_TREE_POSITIONS_H
/* User measurement 2026-11-10, metres from west map boundary.
 * Row Rn is served from Gn. Tree 1 is west; tree 2 is east.
 * First-pass station X aligns chassis centre with tree X (no tool offset yet).
 * No tree Y coordinate or tool reach is inferred from this table. */
static const float full_map_tree_x_m[6][2]={
    {1.240f,2.160f}, /* G1: 124 / 216 cm */
    { .740f,1.640f}, /* G2: 74 / 164 cm */
    { .690f,2.230f}, /* G3: 69 / 223 cm */
    {1.200f,2.110f}, /* G4: 120 / 211 cm */
    {1.310f,2.210f}, /* G5: 131 / 221 cm */
    { .675f,1.885f}  /* G6: 67.5 / 188.5 cm */
};
#endif
