from openglider.utils.table import Table

# Number of columns used to describe a single rod sleeve:
#   chord position (start%/end%) | seam-contour length | centreline length
# Both lengths are measured from termination to termination.
COLS_PER_SLEEVE = 3


def _sleeve_values(sleeve, rib, glider):
    position = f"{sleeve.start_chord * 100:.0f}% / {sleeve.end_chord * 100:.0f}%"
    seam_length = round(1000 * sleeve.get_seam_length(rib, glider=glider), 1)
    center_length = round(1000 * sleeve.get_center_length(rib, glider=glider), 1)
    return position, seam_length, center_length


def get_length_table(glider):
    """
    Build the "rigidfoils" sheet from the rod sleeves (fourreaux de jonc)
    defined in the Airfoil Structure tool.

    For every rib (one row) the extrados sleeves are listed first (HAUT), then
    the intrados sleeves (BAS). Each sleeve reports its chord range and two
    lengths (in millimeters), both measured from termination to termination:
      - "L. couture" : length along the seam contour (the inner edge sewn to
        the panel);
      - "L. milieu"  : length along the centreline of the channel.
    """
    table = Table()
    table.name = "rigidfoils"

    ribs_data = []
    max_upper = 0
    max_lower = 0
    for rib in glider.ribs:
        sleeves = getattr(rib, "rod_sleeves", []) or []
        upper = [s for s in sleeves if s.surface == "extrados"]
        lower = [s for s in sleeves if s.surface == "intrados"]
        upper.sort(key=lambda s: s.start_chord)
        lower.sort(key=lambda s: s.start_chord)
        ribs_data.append((rib, upper, lower))
        max_upper = max(max_upper, len(upper))
        max_lower = max(max_lower, len(lower))

    upper_start = 1
    lower_start = upper_start + max_upper * COLS_PER_SLEEVE

    # group headers (row 0) and column headers (row 1)
    table[0, 0] = "Nervure"
    if max_upper:
        table[0, upper_start] = "HAUT (extrados)"
    if max_lower:
        table[0, lower_start] = "BAS (intrados)"

    def write_headers(block_start, count, label):
        for i in range(count):
            col = block_start + i * COLS_PER_SLEEVE
            table[1, col] = f"{label} {i + 1} corde"
            table[1, col + 1] = "L. couture [mm]"
            table[1, col + 2] = "L. milieu [mm]"

    write_headers(upper_start, max_upper, "Haut")
    write_headers(lower_start, max_lower, "Bas")

    for rib_no, (rib, upper, lower) in enumerate(ribs_data):
        row = rib_no + 2
        table[row, 0] = f"Nervure {rib_no}"

        def write_block(block_start, sleeves):
            for i, sleeve in enumerate(sleeves):
                position, seam_length, total_length = _sleeve_values(
                    sleeve, rib, glider
                )
                col = block_start + i * COLS_PER_SLEEVE
                table[row, col] = position
                table[row, col + 1] = seam_length
                table[row, col + 2] = total_length

        write_block(upper_start, upper)
        write_block(lower_start, lower)

    return table.get_ods_sheet("rigidfoils")
