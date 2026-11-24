"""NASL Word report building blocks (python standard library only).

Extracted on 2026-11-15 from work/reports/review_20261115/build_review_report.py (the review report builder), so
that every engineering report uses the same NASL look: the class Doc writes WordprocessingML paragraphs, headings,
lists, tables, notes, code, figures and the contents field; package() puts the body into a copy of the NASL
template (styles, header and footer with the page background, numbering, theme, fonts and section kept) and
writes the .docx. convert_to_pdf.ps1 then lets Microsoft Word update the contents and export the PDF.
Used by tools/reports/build_engineering_report.py.
"""
import os
import re
import struct
import zipfile
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
TEMPLATE = os.path.join(HERE, 'template_NASL_v1.1.docx')

NAVY, MAGENTA, PURPLE, LILAC = '2B2F7E', 'A64B9C', '6E4FA3', 'EEEAF6'
GRIDC, ZEBRA, INK, GREY, CODEINK = 'C4C4CF', 'F2F2F5', '1F1F26', '5F6070', '3B3566'
TEXT_W = 9026  # dxa between the margins (A4, 1440 left and right)


class Doc:
    def __init__(self, refs, vals=None):
        self.vals = vals or {}    # {name} placeholders in the text
        self.refs = refs          # key -> "Table N" from the previous pass
        self.new_refs = {}
        self.n = {'Table': 0, 'Figure': 0}
        self.parts = []
        self.docpr = 100
        self.nums = []            # numIds of numbered lists to add to numbering.xml
        self.next_num = 20

    def add(self, xml):
        self.parts.append(xml)

    def text(self, s):
        for k, v in self.vals.items():
            s = s.replace('{' + k + '}', str(v))
        return re.sub(r'\{\{([\w:]+)\}\}', lambda m: self.refs.get(m.group(1), '??'), s)

    # runs
    @staticmethod
    def run(t, size=None, color=None, bold=False, italic=False, font=None, spacing=None):
        r = []
        if font:
            r.append(f'<w:rFonts w:ascii="{font}" w:eastAsia="{font}" w:hAnsi="{font}" w:cs="{font}"/>')
        if bold:
            r.append('<w:b/><w:bCs/>')
        if italic:
            r.append('<w:i/><w:iCs/>')
        if color:
            r.append(f'<w:color w:val="{color}"/>')
        if spacing:
            r.append(f'<w:spacing w:val="{spacing}"/>')
        if size:
            r.append(f'<w:sz w:val="{size}"/><w:szCs w:val="{size}"/>')
        rp = f'<w:rPr>{"".join(r)}</w:rPr>' if r else ''
        return f'<w:r>{rp}<w:t xml:space="preserve">{escape(t)}</w:t></w:r>'

    def inline(self, s, size=None, color=None, bold=False, italic=False, code_size=None):
        s = self.text(s)
        out, b = [], bold
        for part in re.split(r'(\*\*|`[^`]*`)', s):
            if part == '**':
                b = not b
            elif len(part) >= 2 and part[0] == '`' and part[-1] == '`':
                cs = code_size or ((size or 21) - 2)
                out.append(self.run(part[1:-1], size=cs, color=CODEINK, font='Consolas'))
            elif part:
                out.append(self.run(part, size=size, color=color, bold=b, italic=italic))
        return ''.join(out)

    @staticmethod
    def para(inner, style=None, keep_next=False, keep_lines=False, page_break=False, num=None,
             pbdr=None, spacing=None, ind=None, jc=None):
        p = []
        if style:
            p.append(f'<w:pStyle w:val="{style}"/>')
        if keep_next:
            p.append('<w:keepNext/>')
        if keep_lines:
            p.append('<w:keepLines/>')
        if page_break:
            p.append('<w:pageBreakBefore/>')
        if num:
            p.append(f'<w:numPr><w:ilvl w:val="{num[1]}"/><w:numId w:val="{num[0]}"/></w:numPr>')
        if pbdr:
            p.append(pbdr)
        if spacing:
            p.append(spacing)
        if ind:
            p.append(ind)
        if jc:
            p.append(f'<w:jc w:val="{jc}"/>')
        ppr = f'<w:pPr>{"".join(p)}</w:pPr>' if p else ''
        return f'<w:p>{ppr}{inner}</w:p>'

    # blocks
    SPACER = '<w:p><w:pPr><w:spacing w:after="80"/></w:pPr></w:p>'

    def h1(self, t):
        # a spacer paragraph right before a page break can spill onto a page of its own
        while self.parts and self.parts[-1] == self.SPACER:
            self.parts.pop()
        self.add(self.para(self.run(t), style='Heading1', page_break=True))

    def h2(self, t):
        self.add(self.para(self.run(t), style='Heading2'))

    def h3(self, t):
        self.add(self.para(self.run(t), style='Heading3'))

    def p(self, s, keep_next=False, size=None, color=None, spacing=None):
        self.add(self.para(self.inline(s, size=size, color=color), keep_next=keep_next,
                           spacing=spacing))

    def lead(self, s):  # a bold run-in line kept with what follows
        self.add(self.para(self.inline(s), style='Strong1', keep_next=True, keep_lines=True,
                           spacing='<w:spacing w:before="200" w:after="40"/>'))

    def meta(self, s):
        self.add(self.para(self.inline(s, size=17, color=GREY, code_size=16), keep_next=True,
                           spacing='<w:spacing w:after="60"/>'))

    def bullets(self, items, level=0):
        for it in items:
            if isinstance(it, tuple) and it[0] == 'bullets':
                self.bullets(it[1], level + 1)
                continue
            self.add(self.para(self.inline(it), style='ListParagraph', num=(2, level),
                               spacing='<w:spacing w:after="70"/>'))

    def numbered(self, items):
        nid = self.next_num
        self.next_num += 1
        self.nums.append(nid)
        for it in items:
            self.add(self.para(self.inline(it), style='ListParagraph', num=(nid, 0),
                               spacing='<w:spacing w:after="70"/>'))

    def blocks(self, items):
        """A list of strings (paragraphs) and ('bullets', [...]) tuples."""
        for it in items:
            if isinstance(it, tuple):
                kind, val = it
                if kind == 'bullets':
                    self.bullets(val)
                else:
                    self.p(val)
            else:
                self.p(it)

    def caption(self, kind, key, s):
        self.n[kind] += 1
        label = f'{kind} {self.n[kind]}'
        self.new_refs[key] = label
        inner = self.run(label + ': ', bold=True) + self.inline(s, code_size=16)
        if kind == 'Table':
            self.add(self.para(inner, style='NASLCaption', keep_next=True, keep_lines=True,
                               spacing='<w:spacing w:before="200" w:after="80"/>'))
        else:
            self.add(self.para(inner, style='NASLCaption', keep_lines=True))

    def note(self, s):
        cell = (f'<w:tc><w:tcPr><w:tcW w:w="{TEXT_W}" w:type="dxa"/><w:tcBorders>'
                '<w:top w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/>'
                f'<w:left w:val="single" w:sz="24" w:space="0" w:color="{NAVY}"/>'
                '<w:bottom w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/>'
                '<w:right w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/></w:tcBorders>'
                '<w:shd w:val="clear" w:color="auto" w:fill="EEF0F8"/><w:tcMar>'
                '<w:top w:w="100" w:type="dxa"/><w:left w:w="180" w:type="dxa"/>'
                '<w:bottom w:w="100" w:type="dxa"/><w:right w:w="140" w:type="dxa"/></w:tcMar>'
                '</w:tcPr>' + self.para(self.inline(s, size=20), spacing='<w:spacing w:after="0"/>')
                + '</w:tc>')
        self.add(self._tbl([TEXT_W], f'<w:tr><w:trPr><w:cantSplit/></w:trPr>{cell}</w:tr>',
                           borders=False))
        self.add(self.para('', spacing='<w:spacing w:after="60"/>'))

    def code(self, s):
        if not s:
            return
        cell = (f'<w:tc><w:tcPr><w:tcW w:w="{TEXT_W}" w:type="dxa"/><w:tcBorders>'
                '<w:top w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/>'
                f'<w:left w:val="single" w:sz="12" w:space="0" w:color="{PURPLE}"/>'
                '<w:bottom w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/>'
                '<w:right w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/></w:tcBorders>'
                '<w:shd w:val="clear" w:color="auto" w:fill="F4F4F8"/><w:tcMar>'
                '<w:top w:w="70" w:type="dxa"/><w:left w:w="140" w:type="dxa"/>'
                '<w:bottom w:w="70" w:type="dxa"/><w:right w:w="120" w:type="dxa"/></w:tcMar>'
                '</w:tcPr>' + self.para(self.run(s, size=15, color=CODEINK, font='Consolas'),
                                        spacing='<w:spacing w:after="0" w:line="228" w:lineRule="auto"/>')
                + '</w:tc>')
        self.add(self._tbl([TEXT_W], f'<w:tr><w:trPr><w:cantSplit/></w:trPr>{cell}</w:tr>',
                           borders=False))
        self.add(self.para('', spacing='<w:spacing w:after="60"/>'))

    # tables
    @staticmethod
    def _tbl(widths, rows_xml, borders=True):
        b = (f'<w:top w:val="single" w:sz="4" w:space="0" w:color="{GRIDC}"/>'
             f'<w:left w:val="single" w:sz="4" w:space="0" w:color="{GRIDC}"/>'
             f'<w:bottom w:val="single" w:sz="4" w:space="0" w:color="{GRIDC}"/>'
             f'<w:right w:val="single" w:sz="4" w:space="0" w:color="{GRIDC}"/>'
             f'<w:insideH w:val="single" w:sz="4" w:space="0" w:color="{GRIDC}"/>'
             f'<w:insideV w:val="single" w:sz="4" w:space="0" w:color="{GRIDC}"/>')
        if not borders:
            b = ''.join(f'<w:{e} w:val="none" w:sz="0" w:space="0" w:color="FFFFFF"/>'
                        for e in ['top', 'left', 'bottom', 'right', 'insideH', 'insideV'])
        grid = ''.join(f'<w:gridCol w:w="{w}"/>' for w in widths)
        return (f'<w:tbl><w:tblPr><w:tblW w:w="{sum(widths)}" w:type="dxa"/><w:tblBorders>{b}'
                '</w:tblBorders><w:tblLayout w:type="fixed"/><w:tblCellMar><w:left w:w="10" '
                'w:type="dxa"/><w:right w:w="10" w:type="dxa"/></w:tblCellMar><w:tblLook '
                'w:val="0000" w:firstRow="0" w:lastRow="0" w:firstColumn="0" w:lastColumn="0" '
                f'w:noHBand="0" w:noVBand="0"/></w:tblPr><w:tblGrid>{grid}</w:tblGrid>{rows_xml}'
                '</w:tbl>')

    def _cell(self, c, w, fill, size, keep, valign, pad, header=False):
        """c: str, or dict(paras=[str | dict(text, size, color, bold, bullet)], fill, color, bold, jc)."""
        if not isinstance(c, dict):
            c = {'paras': [c]}
        paras = c.get('paras', [''])
        fill = c.get('fill', fill)
        color = c.get('color', 'FFFFFF' if header else None)
        bold = c.get('bold', header)
        jc = c.get('jc')
        out = []
        for i, pp in enumerate(paras):
            if not isinstance(pp, dict):
                pp = {'text': pp}
            last = i == len(paras) - 1
            sp = (f'<w:spacing w:after="{0 if last else 40}" w:line="252" w:lineRule="auto"/>')
            ind = None
            psize = pp.get('size', size)
            pcolor = pp.get('color', color)
            inner = self.inline(pp.get('text', ''), size=psize, color=pcolor,
                                bold=pp.get('bold', bold), code_size=psize - 2)
            if pp.get('bullet'):
                ind = '<w:ind w:left="200" w:hanging="160"/>'
                inner = (self.run('•', size=psize, color=pcolor or PURPLE)
                         + '<w:r><w:tab/></w:r>' + inner)
            out.append(self.para(inner, keep_next=keep, keep_lines=True, spacing=sp, ind=ind,
                                 jc=jc))
        tcpr = (f'<w:tcPr><w:tcW w:w="{w}" w:type="dxa"/><w:shd w:val="clear" w:color="auto" '
                f'w:fill="{fill}"/><w:tcMar><w:top w:w="{pad[0]}" w:type="dxa"/><w:left '
                f'w:w="{pad[1]}" w:type="dxa"/><w:bottom w:w="{pad[0]}" w:type="dxa"/><w:right '
                f'w:w="{pad[1]}" w:type="dxa"/></w:tcMar><w:vAlign w:val="{valign}"/></w:tcPr>')
        return f'<w:tc>{tcpr}{"".join(out)}</w:tc>'

    def table(self, header, rows, widths, size=19, valign='top', pad=(50, 90), zebra=True,
              keep_all=None):
        assert sum(widths) <= TEXT_W, (sum(widths), header)
        if keep_all is None:
            keep_all = len(rows) <= 8
        xml = []
        if header:
            cells = ''.join(self._cell(h, w, NAVY, size, True, 'center', pad, header=True)
                            for h, w in zip(header, widths))
            xml.append(f'<w:tr><w:trPr><w:cantSplit/><w:tblHeader/></w:trPr>{cells}</w:tr>')
        for i, r in enumerate(rows):
            fill = ZEBRA if (zebra and i % 2 == 1) else 'FFFFFF'
            keep = keep_all and i < len(rows) - 1
            cells = ''.join(self._cell(c, w, fill, size, keep, valign, pad) for c, w in zip(r, widths))
            xml.append(f'<w:tr><w:trPr><w:cantSplit/></w:trPr>{cells}</w:tr>')
        self.add(self._tbl(widths, ''.join(xml)))
        self.add(self.SPACER)

    def kv_table(self, rows, widths=(2346, 6680), size=20, pad=(70, 120), keep=False):
        xml = []
        for i, (k, v) in enumerate(rows):
            if v.startswith('[TBD'):
                v = {'paras': [v], 'color': MAGENTA, 'bold': True}
            kp = keep and i < len(rows) - 1
            cells = (self._cell({'paras': [k], 'color': NAVY, 'bold': True}, widths[0], LILAC, size,
                                kp, 'center', pad)
                     + self._cell(v, widths[1], 'FFFFFF', size, kp, 'center', pad))
            xml.append(f'<w:tr><w:trPr><w:cantSplit/></w:trPr>{cells}</w:tr>')
        self.add(self._tbl(list(widths), ''.join(xml)))

    def figure(self, rid, path, width_cm, name, descr):
        with open(path, 'rb') as f:
            head = f.read(24)
        w, h = struct.unpack('>II', head[16:24])
        cx = int(width_cm * 360000)
        cy = int(cx * h / w)
        self.docpr += 1
        self.add(self.inline_image(rid, cx, cy, self.docpr, name, descr, keep_next=True))

    @staticmethod
    def inline_image(rid, cx, cy, docpr, name, descr, keep_next=False, spacing_after=60):
        a = 'http://schemas.openxmlformats.org/drawingml/2006/main'
        pic = 'http://schemas.openxmlformats.org/drawingml/2006/picture'
        d = (f'<w:r><w:rPr><w:noProof/></w:rPr><w:drawing><wp:inline distT="0" distB="0" distL="0" '
             f'distR="0"><wp:extent cx="{cx}" cy="{cy}"/><wp:effectExtent l="0" t="0" r="0" b="0"/>'
             f'<wp:docPr id="{docpr}" name="{escape(name)}" descr="{escape(descr)}"/>'
             f'<wp:cNvGraphicFramePr><a:graphicFrameLocks xmlns:a="{a}" noChangeAspect="1"/>'
             f'</wp:cNvGraphicFramePr><a:graphic xmlns:a="{a}"><a:graphicData uri="{pic}">'
             f'<pic:pic xmlns:pic="{pic}"><pic:nvPicPr><pic:cNvPr id="0" name="{escape(name)}"/>'
             '<pic:cNvPicPr><a:picLocks noChangeAspect="1" noChangeArrowheads="1"/></pic:cNvPicPr>'
             f'</pic:nvPicPr><pic:blipFill><a:blip r:embed="{rid}"/><a:srcRect/><a:stretch>'
             '<a:fillRect/></a:stretch></pic:blipFill><pic:spPr bwMode="auto"><a:xfrm><a:off x="0" '
             f'y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/>'
             '</a:prstGeom></pic:spPr></pic:pic></a:graphicData></a:graphic></wp:inline>'
             '</w:drawing></w:r>')
        sp = f'<w:spacing w:after="{spacing_after}"/>'
        return Doc.para(d, keep_next=keep_next, spacing=sp, jc='center' if keep_next else None)

    def toc(self):
        x = ('<w:sdt><w:sdtPr><w:alias w:val="Contents"/><w:docPartObj><w:docPartGallery '
             'w:val="Table of Contents"/><w:docPartUnique/></w:docPartObj></w:sdtPr><w:sdtContent>'
             '<w:p><w:pPr><w:pStyle w:val="TOC1"/><w:tabs><w:tab w:val="right" w:leader="dot" '
             'w:pos="9016"/></w:tabs></w:pPr><w:r><w:fldChar w:fldCharType="begin" '
             'w:dirty="true"/></w:r><w:r><w:instrText xml:space="preserve"> TOC \\o "1-2" \\h \\z '
             '\\u </w:instrText></w:r><w:r><w:fldChar w:fldCharType="separate"/></w:r><w:r><w:t>'
             'The contents are filled in when Word updates the fields.</w:t></w:r><w:r><w:fldChar '
             'w:fldCharType="end"/></w:r></w:p></w:sdtContent></w:sdt>')
        self.add(x)


# ------------------------------------------------------------------ package
OLD_FOOTER = 'NASL Technologies | Shaddad UGV Real Robot and Model (MBD) | Internal'


def package(d, out, meta, images):
    """Write the .docx: d = Doc, out = path, meta = dict(title, subject, keywords, description, date, footer),
    images = list of (rid, media_name, path) for the body figures (rid from rId101 up)."""
    z = zipfile.ZipFile(TEMPLATE)
    tdoc = z.read('word/document.xml').decode('utf-8')
    head = tdoc[:tdoc.index('<w:body>') + len('<w:body>')]
    sect = tdoc[tdoc.rindex('<w:sectPr'):]
    assert sect.endswith('</w:body></w:document>')
    body = ''.join(d.parts)
    docxml = head + body + sect
    ET.fromstring(docxml.encode('utf-8'))  # well-formed check

    rels = z.read('word/_rels/document.xml.rels').decode('utf-8')
    keep = []
    for m in re.finditer(r'<Relationship [^>]*/>', rels):
        rel = m.group(0)
        if '/image"' in rel and 'Target="media/image1.png"' not in rel:
            continue
        keep.append(rel)
    for rid, media, _ in images:
        keep.append(f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/'
                    f'2006/relationships/image" Target="media/{media}"/>')
    relxml = rels[:rels.index('<Relationship ')] + ''.join(keep) + '</Relationships>'
    ET.fromstring(relxml.encode('utf-8'))

    num = z.read('word/numbering.xml').decode('utf-8')
    extra = ''.join(f'<w:num w:numId="{n}"><w:abstractNumId w:val="2"/><w:lvlOverride w:ilvl="0">'
                    '<w:startOverride w:val="1"/></w:lvlOverride></w:num>' for n in d.nums)
    cut = num.find('<w:numIdMacAtCleanup')
    cut = cut if cut >= 0 else num.rindex('</w:numbering>')
    num = num[:cut] + extra + num[cut:]
    ET.fromstring(num.encode('utf-8'))

    footer = z.read('word/footer1.xml').decode('utf-8')
    assert OLD_FOOTER in footer
    footer = footer.replace(OLD_FOOTER, escape(meta['footer']))

    core = z.read('docProps/core.xml').decode('utf-8')
    stamp = meta['date'] + 'T00:00:00Z'
    for tag, val in [('dc:title', meta['title']), ('dc:subject', meta.get('subject', '')),
                     ('cp:keywords', meta.get('keywords', '')), ('dc:description', meta.get('description', '')),
                     ('cp:lastModifiedBy', 'NASL Technologies'), ('cp:revision', '1')]:
        core = re.sub(rf'<{tag}>.*?</{tag}>', f'<{tag}>{escape(val)}</{tag}>', core, flags=re.S)
    core = re.sub(r'(<dcterms:created[^>]*>).*?(</dcterms:created>)', lambda m: m.group(1) + stamp + m.group(2), core)
    core = re.sub(r'(<dcterms:modified[^>]*>).*?(</dcterms:modified>)', lambda m: m.group(1) + stamp + m.group(2), core)

    replace = {'word/document.xml': docxml, 'word/_rels/document.xml.rels': relxml,
               'word/numbering.xml': num, 'word/footer1.xml': footer, 'docProps/core.xml': core}
    tmp = out + '.tmp'
    with zipfile.ZipFile(tmp, 'w', zipfile.ZIP_DEFLATED) as zo:
        for info in z.infolist():
            n = info.filename
            if n.startswith('word/media/') and n not in ('word/media/image1.png', 'word/media/image38.png'):
                continue
            zo.writestr(n, replace[n].encode('utf-8') if n in replace else z.read(n))
        for _, media, path in images:
            with open(path, 'rb') as f:
                zo.writestr('word/media/' + media, f.read())
    os.replace(tmp, out)
    used = set(re.findall(r'r:(?:embed|id)="([^"]+)"', docxml))
    have = set(re.findall(r'Id="([^"]+)"', relxml))
    assert used <= have, used - have
    return len(body)
