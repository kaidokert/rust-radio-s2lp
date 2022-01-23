from string import Template

tmpl = '''
#[bitfield]
#[derive(Copy, Clone, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub struct ${regname} {
    pub raw: u8,
}

impl radio::Register for ${regname} {
    const ADDRESS: u8 = ${regvalue}u8;
    type Word = u8;
    type Error = Infallible;
}'''

with open('s2lptrim.h') as f:
    for line in f:
        stripped = line.strip()
        # print(stripped)
        regname = stripped[8:27].strip()
        regval = stripped[27:].strip()
        regs = regname.lower().split('_')
        joined = ''.join([ x[:1].upper() + x[1:] for x in regs])
        tmppS = Template(tmpl)
        foo = tmppS.substitute(regname=joined,regvalue=regval)
        # print(regname,joined, regval)
        print(foo)