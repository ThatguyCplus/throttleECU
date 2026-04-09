import re

sch_file = 'f411testingreliablitymotor/c2000_f280049_throttle/throttleecupcb/TasaruV0.0.1/TasaruV0.0.1.kicad_sch'

with open(sch_file, 'r') as f:
    content = f.read()

# U1 instances: unit -> (sym_x, sym_y, angle)
u1_pos = {
    3: (222.25, 69.85, 0),
    5: (195.58, 276.86, 0),
    4: (528.32, 52.07, 0),
    2: (528.32, 115.57, 0),
    1: (528.32, 184.15, 0),
}

pin_defs = {
    1: [
        ('1','GPIO28',-17.78,-27.94,0,5.08),('50','GPIO13',-17.78,2.54,0,5.08),
        ('51','GPIO12',-17.78,5.08,0,5.08),('52','GPIO11',-17.78,7.62,0,5.08),
        ('54','GPIO16',-17.78,-5.08,0,5.08),('55','GPIO17',-17.78,-7.62,0,5.08),
        ('56','GPIO24',-17.78,-17.78,0,5.08),('57','GPIO25',-17.78,-20.32,0,5.08),
        ('58','GPIO26',-17.78,-22.86,0,5.08),('59','GPIO27',-17.78,-25.4,0,5.08),
        ('68','GPIO18_X2',-17.78,-10.16,0,5.08),('74','GPIO08',-17.78,15.24,0,5.08),
        ('75','GPIO04',-17.78,25.4,0,5.08),('76','GPIO03',-17.78,27.94,0,5.08),
        ('77','GPIO02',-17.78,30.48,0,5.08),('78','GPIO01',-17.78,33.02,0,5.08),
        ('79','GPIO00',-17.78,35.56,0,5.08),('81','GPIO23_SW',-17.78,-15.24,0,5.08),
        ('83','GPIO22_SW',-17.78,-12.7,0,5.08),('84','GPIO07',-17.78,17.78,0,5.08),
        ('89','GPIO05',-17.78,22.86,0,5.08),('90','GPIO09',-17.78,12.7,0,5.08),
        ('93','GPIO10',-17.78,10.16,0,5.08),('95','GPIO15',-17.78,-2.54,0,5.08),
        ('96','GPIO14',-17.78,0,0,5.08),('97','GPIO06',-17.78,20.32,0,5.08),
        ('98','GPIO30',-17.78,-33.02,0,5.08),('99','GPIO31',-17.78,-35.56,0,5.08),
        ('100','GPIO29',-17.78,-30.48,0,5.08),
    ],
    2: [
        ('53','GPIO33',-17.78,10.16,0,5.08),('61','GPIO37_TDO',-17.78,2.54,0,5.08),
        ('63','GPIO35_TDI',-17.78,5.08,0,5.08),('64','GPIO32',-17.78,12.7,0,5.08),
        ('65','GPIO56',-17.78,-5.08,0,5.08),('66','GPIO57',-17.78,-7.62,0,5.08),
        ('67','GPIO58',-17.78,-10.16,0,5.08),('85','GPIO40',-17.78,-2.54,0,5.08),
        ('91','GPIO39',-17.78,0,0,5.08),('92','GPIO59',-17.78,-12.7,0,5.08),
        ('94','GPIO34',-17.78,7.62,0,5.08),
    ],
    3: [
        ('6','A6_PGA5_OF',-50.8,12.7,0,5.08),('7','B2_C6_PGA3_OF',-50.8,-5.08,0,5.08),
        ('8','B3_VDAC',-50.8,-7.62,0,5.08),('9','A2_B6_PGA1_OF',-50.8,22.86,0,5.08),
        ('10','A3',-50.8,20.32,0,5.08),('13','PGA5_GND',50.8,-2.54,180,5.08),
        ('14','PGA1_GND',50.8,2.54,180,5.08),('15','PGA3_GND',50.8,0,180,5.08),
        ('16','PGA5_IN',50.8,17.78,180,5.08),('17','C4',-50.8,-25.4,0,5.08),
        ('18','PGA1_IN',50.8,27.94,180,5.08),('19','C0',-50.8,-17.78,0,5.08),
        ('20','PGA3_IN',50.8,22.86,180,5.08),('21','C2',-50.8,-22.86,0,5.08),
        ('22','A1_DACB_OUT',-50.8,25.4,0,5.08),('23','A0_B15_C15_DACA_OUT',-50.8,27.94,0,5.08),
        ('24','VREFHIB_VREFHIC',50.8,-17.78,180,5.08),('25','VREFHIA',50.8,-15.24,180,5.08),
        ('26','VREFLOB_VREFLOC',50.8,-27.94,180,5.08),('27','VREFLOA',50.8,-25.4,180,5.08),
        ('28','C5_PGA6_IN',50.8,15.24,180,5.08),('29','C1',-50.8,-20.32,0,5.08),
        ('30','PGA2_IN',50.8,25.4,180,5.08),('31','C3_PGA4_IN',50.8,20.32,180,5.08),
        ('32','PGA2_GND_PGA4_GND_PGA6_GND',50.8,-5.08,180,5.08),
        ('35','A5',-50.8,15.24,0,5.08),('36','A4_B8_PGA2_OF',-50.8,17.78,0,5.08),
        ('37','A8_PGA6_OF',-50.8,10.16,0,5.08),('38','A9',-50.8,7.62,0,5.08),
        ('39','B4_C8_PGA4_OF',-50.8,-10.16,0,5.08),('40','B1_A10_C10_PGA7_OF',-50.8,-2.54,0,5.08),
        ('41','B0',-50.8,0,0,5.08),('42','PGA7_GND',50.8,-7.62,180,5.08),
        ('43','PGA7_IN',50.8,12.7,180,5.08),('44','C14',-50.8,-27.94,0,5.08),
    ],
    4: [
        ('2','XRS_N',-20.32,-2.54,0,5.08),('48','FLT2',20.32,-2.54,180,5.08),
        ('49','FLT1',20.32,2.54,180,5.08),('60','TCK',-20.32,7.62,0,5.08),
        ('62','TMS',-20.32,2.54,0,5.08),('69','X1',-20.32,-7.62,0,5.08),
        ('73','VREGENZ',20.32,7.62,180,5.08),
    ],
    5: [
        ('3','VDDIO',-27.94,-5.08,0,5.08),('4','VDD',-27.94,15.24,0,5.08),
        ('5','VSS',27.94,15.24,180,5.08),('11','VDDA',-27.94,2.54,0,5.08),
        ('12','VSSA',27.94,2.54,180,5.08),('33','VSSA',27.94,0,180,5.08),
        ('34','VDDA',-27.94,0,0,5.08),('45','VSS',27.94,12.7,180,5.08),
        ('46','VDD',-27.94,12.7,0,5.08),('47','VDDIO',-27.94,-7.62,0,5.08),
        ('70','VDDIO',-27.94,-10.16,0,5.08),('71','VDD',-27.94,10.16,0,5.08),
        ('72','VSS',27.94,10.16,180,5.08),('80','VDDIO_SW',-27.94,-17.78,0,5.08),
        ('82','VSS_SW',27.94,-17.78,180,5.08),('86','VSS',27.94,7.62,180,5.08),
        ('87','VDD',-27.94,7.62,0,5.08),('88','VDDIO',-27.94,-12.7,0,5.08),
    ],
}

# Compute pin endpoints in schematic coordinates
pin_endpoints = {}
for unit, pins in pin_defs.items():
    sx, sy, sa = u1_pos[unit]
    for pnum, pname, px, py, pa, pl in pins:
        ex = round(sx + px, 2)
        ey = round(sy + py, 2)
        pin_endpoints[pnum] = (ex, ey, pname)

# Collect no-connect markers
no_connects = set()
for m in re.finditer(r'\(no_connect\s*\n\s*\(at\s+([\d.]+)\s+([\d.]+)', content):
    no_connects.add((float(m.group(1)), float(m.group(2))))

# Collect labels
labels = {}
for m in re.finditer(r'\((label|global_label|net_label|hierarchical_label)\s+"([^"]+)"\s*\n\s*\(at\s+([\d.]+)\s+([\d.]+)', content):
    pos = (float(m.group(3)), float(m.group(4)))
    labels[pos] = m.group(2)

# Collect power symbols
power_syms = []
for m in re.finditer(r'\(symbol\s*\n\s*\(lib_id "power:([^"]+)"\)\s*\n\s*\(at\s+([\d.]+)\s+([\d.]+)', content):
    power_syms.append((m.group(1), float(m.group(2)), float(m.group(3))))

# Collect wires
wires = []
for m in re.finditer(r'\(wire\s*\n\s*\(pts\s+\(xy\s+([\d.]+)\s+([\d.]+)\)\s+\(xy\s+([\d.]+)\s+([\d.]+)\)', content):
    wires.append(((float(m.group(1)), float(m.group(2))), (float(m.group(3)), float(m.group(4)))))

TOLERANCE = 0.05

def find_at_nc(x, y):
    for ix, iy in no_connects:
        if abs(ix - x) < TOLERANCE and abs(iy - y) < TOLERANCE:
            return True
    return False

def find_label_at(x, y):
    for (lx, ly), name in labels.items():
        if abs(lx - x) < TOLERANCE and abs(ly - y) < TOLERANCE:
            return name
    return None

def find_power_at(x, y):
    for name, px, py in power_syms:
        if abs(px - x) < TOLERANCE and abs(py - y) < TOLERANCE:
            return name
    return None

def find_wire_endpoint(x, y):
    results = []
    for (w1x, w1y), (w2x, w2y) in wires:
        if abs(w1x - x) < TOLERANCE and abs(w1y - y) < TOLERANCE:
            results.append((w2x, w2y))
        elif abs(w2x - x) < TOLERANCE and abs(w2y - y) < TOLERANCE:
            results.append((w1x, w1y))
    return results

def trace_net(x, y, visited=None, depth=0):
    if visited is None:
        visited = set()
    key = (round(x, 2), round(y, 2))
    if key in visited or depth > 20:
        return []
    visited.add(key)

    results = []

    lbl = find_label_at(x, y)
    if lbl:
        results.append(('label', lbl))

    pwr = find_power_at(x, y)
    if pwr:
        results.append(('power', pwr))

    if find_at_nc(x, y):
        results.append(('no_connect', 'X'))

    other_ends = find_wire_endpoint(x, y)
    for ox, oy in other_ends:
        results.extend(trace_net(ox, oy, visited, depth + 1))

    return results

# Classify each pin
print('=' * 80)
print('U1 (F280049CPZS) PIN CONNECTION ANALYSIS')
print('=' * 80)

gpio_connected = {}
gpio_noconnect = {}
gpio_unconnected = {}
other_connected = {}
other_noconnect = {}
other_unconnected = {}

for pnum in sorted(pin_endpoints.keys(), key=lambda x: int(x)):
    ex, ey, pname = pin_endpoints[pnum]
    net_info = trace_net(ex, ey)

    is_gpio = pname.startswith('GPIO')

    has_nc = any(t == 'no_connect' for t, n in net_info)
    real_nets = [(t, n) for t, n in net_info if t != 'no_connect']

    if has_nc:
        if is_gpio:
            gpio_noconnect[pnum] = (pname, ex, ey)
        else:
            other_noconnect[pnum] = (pname, ex, ey)
    elif real_nets:
        net_str = ', '.join([f'{n}' for t, n in real_nets])
        if is_gpio:
            gpio_connected[pnum] = (pname, net_str, ex, ey)
        else:
            other_connected[pnum] = (pname, net_str, ex, ey)
    else:
        if is_gpio:
            gpio_unconnected[pnum] = (pname, ex, ey)
        else:
            other_unconnected[pnum] = (pname, ex, ey)

print('\n--- GPIO PINS WITH NET CONNECTIONS ---')
for pnum in sorted(gpio_connected.keys(), key=lambda x: int(x)):
    pname, net_str, ex, ey = gpio_connected[pnum]
    print(f'  Pin {pnum:>3s} ({pname:>15s}) -> {net_str}  [at ({ex}, {ey})]')

print(f'\n--- GPIO PINS MARKED NO-CONNECT (X) ---')
for pnum in sorted(gpio_noconnect.keys(), key=lambda x: int(x)):
    pname, ex, ey = gpio_noconnect[pnum]
    print(f'  Pin {pnum:>3s} ({pname:>15s})  [at ({ex}, {ey})]')

print(f'\n--- GPIO PINS UNCONNECTED (no wire/label/NC found) ---')
for pnum in sorted(gpio_unconnected.keys(), key=lambda x: int(x)):
    pname, ex, ey = gpio_unconnected[pnum]
    print(f'  Pin {pnum:>3s} ({pname:>15s})  [at ({ex}, {ey})]')

print(f'\n--- NON-GPIO PINS WITH NET CONNECTIONS ---')
for pnum in sorted(other_connected.keys(), key=lambda x: int(x)):
    pname, net_str, ex, ey = other_connected[pnum]
    print(f'  Pin {pnum:>3s} ({pname:>30s}) -> {net_str}')

print(f'\n--- NON-GPIO PINS MARKED NO-CONNECT ---')
for pnum in sorted(other_noconnect.keys(), key=lambda x: int(x)):
    pname, ex, ey = other_noconnect[pnum]
    print(f'  Pin {pnum:>3s} ({pname:>30s})')

print(f'\n--- NON-GPIO PINS UNCONNECTED ---')
for pnum in sorted(other_unconnected.keys(), key=lambda x: int(x)):
    pname, ex, ey = other_unconnected[pnum]
    print(f'  Pin {pnum:>3s} ({pname:>30s})  [at ({ex}, {ey})]')

print(f'\n{"="*80}')
print(f'SUMMARY')
print(f'{"="*80}')
print(f'  GPIO pins with nets:      {len(gpio_connected)}')
print(f'  GPIO pins no-connect:     {len(gpio_noconnect)}')
print(f'  GPIO pins unconnected:    {len(gpio_unconnected)}')
print(f'  Non-GPIO pins with nets:  {len(other_connected)}')
print(f'  Non-GPIO pins no-connect: {len(other_noconnect)}')
print(f'  Non-GPIO pins unconnected:{len(other_unconnected)}')
print(f'  Total pins:               {len(pin_endpoints)}')
