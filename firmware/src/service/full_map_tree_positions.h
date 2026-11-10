#ifndef MANGO_FULL_MAP_TREE_POSITIONS_H
#define MANGO_FULL_MAP_TREE_POSITIONS_H
/* User measurement 2026-11-11, metres from west map boundary.
 * Row Rn is served from Gn. Tree 1 is west; tree 2 is east.
 * First-pass station X aligns chassis centre with tree X (no tool offset yet).
 * No tree Y coordinate or tool reach is inferred from this table. */
static const float full_map_tree_x_m[6][2]={
    {1.230f,2.150f}, /* G1: 123 / 215 cm */
    { .730f,1.630f}, /* G2: 73 / 163 cm */
    { .680f,2.220f}, /* G3: 68 / 222 cm */
    {1.190f,2.100f}, /* G4: 119 / 210 cm */
    {1.300f,2.200f}, /* G5: 130 / 220 cm */
    { .665f,1.875f}  /* G6: 66.5 / 187.5 cm */
};
#endif
