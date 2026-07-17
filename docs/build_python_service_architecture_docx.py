import html
import os
import re
import zipfile


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MD_PATH = os.path.join(BASE_DIR, "光伏清扫机器人当前软件架构.md")
DOCX_PATH = os.path.join(BASE_DIR, "光伏清扫机器人当前软件架构.docx")

W_NS = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
R_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"


def esc(text):
    return html.escape(text, quote=True)


def text_runs(text):
    parts = re.split(r"(`[^`]+`)", text)
    runs = []
    for part in parts:
        if not part:
            continue
        if part.startswith("`") and part.endswith("`"):
            runs.append(
                '<w:r><w:rPr><w:rFonts w:ascii="Consolas" w:eastAsia="Microsoft YaHei" />'
                '<w:sz w:val="20" /><w:color w:val="374151" /></w:rPr>'
                '<w:t xml:space="preserve">%s</w:t></w:r>' % esc(part[1:-1])
            )
        else:
            runs.append('<w:r><w:t xml:space="preserve">%s</w:t></w:r>' % esc(part))
    return "".join(runs)


def paragraph(text="", style=None, num_id=None, level=0, keep_next=False):
    ppr = []
    if style:
        ppr.append('<w:pStyle w:val="%s" />' % style)
    if keep_next:
        ppr.append("<w:keepNext />")
    if num_id is not None:
        ppr.append(
            '<w:numPr><w:ilvl w:val="%s" /><w:numId w:val="%s" /></w:numPr>'
            % (level, num_id)
        )
    ppr_xml = "<w:pPr>%s</w:pPr>" % "".join(ppr) if ppr else ""
    return "<w:p>%s%s</w:p>" % (ppr_xml, text_runs(text))


def table(rows):
    if not rows:
        return ""
    col_count = max(len(row) for row in rows)
    if col_count == 2:
        widths = [2200, 7160]
    elif col_count == 3:
        widths = [2200, 3580, 3580]
    else:
        base = int(9360 / col_count)
        widths = [base] * col_count
        widths[-1] = 9360 - base * (col_count - 1)

    grid = "".join('<w:gridCol w:w="%s" />' % width for width in widths)
    xml = [
        '<w:tbl>',
        '<w:tblPr>',
        '<w:tblW w:w="9360" w:type="dxa" />',
        '<w:tblInd w:w="120" w:type="dxa" />',
        '<w:tblBorders>',
        '<w:top w:val="single" w:sz="4" w:space="0" w:color="D0D7DE" />',
        '<w:left w:val="single" w:sz="4" w:space="0" w:color="D0D7DE" />',
        '<w:bottom w:val="single" w:sz="4" w:space="0" w:color="D0D7DE" />',
        '<w:right w:val="single" w:sz="4" w:space="0" w:color="D0D7DE" />',
        '<w:insideH w:val="single" w:sz="4" w:space="0" w:color="D0D7DE" />',
        '<w:insideV w:val="single" w:sz="4" w:space="0" w:color="D0D7DE" />',
        '</w:tblBorders>',
        '<w:tblCellMar><w:top w:w="100" w:type="dxa" /><w:left w:w="120" w:type="dxa" />'
        '<w:bottom w:w="100" w:type="dxa" /><w:right w:w="120" w:type="dxa" /></w:tblCellMar>',
        '</w:tblPr>',
        '<w:tblGrid>%s</w:tblGrid>' % grid,
    ]

    for index, row in enumerate(rows):
        xml.append("<w:tr>")
        for col_index in range(col_count):
            cell = row[col_index] if col_index < len(row) else ""
            fill = '<w:shd w:fill="F2F4F7" />' if index == 0 else ""
            bold = '<w:b />' if index == 0 else ""
            xml.append(
                '<w:tc><w:tcPr><w:tcW w:w="%s" w:type="dxa" />%s'
                '<w:vAlign w:val="center" /></w:tcPr>'
                '<w:p><w:pPr><w:spacing w:before="0" w:after="0" w:line="280" w:lineRule="auto" /></w:pPr>'
                '<w:r><w:rPr>%s</w:rPr><w:t xml:space="preserve">%s</w:t></w:r></w:p></w:tc>'
                % (widths[col_index], fill, bold, esc(cell))
            )
        xml.append("</w:tr>")
    xml.append("</w:tbl>")
    xml.append(paragraph(""))
    return "".join(xml)


def parse_markdown(md_text):
    body = []
    table_rows = []
    in_table = False

    def flush_table():
        nonlocal table_rows, in_table
        if table_rows:
            body.append(table(table_rows))
            table_rows = []
        in_table = False

    for raw_line in md_text.splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped:
            flush_table()
            continue

        if stripped.startswith("|") and stripped.endswith("|"):
            cells = [cell.strip() for cell in stripped.strip("|").split("|")]
            if all(re.fullmatch(r":?-{3,}:?", cell or "") for cell in cells):
                in_table = True
                continue
            table_rows.append(cells)
            in_table = True
            continue

        flush_table()
        if stripped.startswith("# "):
            body.append(paragraph(stripped[2:].strip(), "DocTitle"))
            body.append(paragraph("技术栈、系统架构与核心功能梳理", "DocSubtitle"))
        elif stripped.startswith("## "):
            body.append(paragraph(stripped[3:].strip(), "Heading1", keep_next=True))
        elif stripped.startswith("### "):
            body.append(paragraph(stripped[4:].strip(), "Heading2", keep_next=True))
        elif re.match(r"^\d+\.\s+", stripped):
            body.append(paragraph(re.sub(r"^\d+\.\s+", "", stripped), "ListParagraph", num_id=2))
        elif stripped.startswith("- "):
            body.append(paragraph(stripped[2:].strip(), "ListParagraph", num_id=1))
        else:
            body.append(paragraph(stripped, "BodyText"))

    flush_table()
    return "".join(body)


def content_types_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/word/numbering.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.numbering+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>"""


def root_rels_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>"""


def document_rels_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/numbering" Target="numbering.xml"/>
</Relationships>"""


def styles_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="%s">
  <w:style w:type="paragraph" w:default="1" w:styleId="Normal">
    <w:name w:val="Normal"/>
    <w:qFormat/>
    <w:pPr><w:spacing w:before="0" w:after="120" w:line="264" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/><w:color w:val="111827"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="BodyText">
    <w:name w:val="Body Text"/>
    <w:basedOn w:val="Normal"/>
    <w:pPr><w:spacing w:before="0" w:after="120" w:line="264" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/><w:color w:val="111827"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="DocTitle">
    <w:name w:val="Document Title"/>
    <w:basedOn w:val="Normal"/>
    <w:pPr><w:spacing w:before="0" w:after="120"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:b/><w:sz w:val="44"/><w:color w:val="0B2545"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="DocSubtitle">
    <w:name w:val="Document Subtitle"/>
    <w:basedOn w:val="Normal"/>
    <w:pPr><w:spacing w:before="0" w:after="240"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/><w:color w:val="4B5563"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading1">
    <w:name w:val="Heading 1"/>
    <w:basedOn w:val="Normal"/>
    <w:qFormat/>
    <w:pPr><w:outlineLvl w:val="0"/><w:spacing w:before="320" w:after="160"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:b/><w:sz w:val="32"/><w:color w:val="2E74B5"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading2">
    <w:name w:val="Heading 2"/>
    <w:basedOn w:val="Normal"/>
    <w:qFormat/>
    <w:pPr><w:outlineLvl w:val="1"/><w:spacing w:before="240" w:after="120"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:b/><w:sz w:val="26"/><w:color w:val="2E74B5"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="ListParagraph">
    <w:name w:val="List Paragraph"/>
    <w:basedOn w:val="Normal"/>
    <w:pPr><w:spacing w:before="0" w:after="160" w:line="280" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Calibri" w:hAnsi="Calibri" w:eastAsia="Microsoft YaHei"/><w:sz w:val="22"/></w:rPr>
  </w:style>
</w:styles>""" % W_NS


def numbering_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:numbering xmlns:w="%s">
  <w:abstractNum w:abstractNumId="1">
    <w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="bullet"/><w:lvlText w:val="•"/>
      <w:pPr><w:ind w:left="720" w:hanging="360"/></w:pPr></w:lvl>
  </w:abstractNum>
  <w:num w:numId="1"><w:abstractNumId w:val="1"/></w:num>
  <w:abstractNum w:abstractNumId="2">
    <w:lvl w:ilvl="0"><w:start w:val="1"/><w:numFmt w:val="decimal"/><w:lvlText w:val="%%1."/>
      <w:pPr><w:ind w:left="720" w:hanging="360"/></w:pPr></w:lvl>
  </w:abstractNum>
  <w:num w:numId="2"><w:abstractNumId w:val="2"/></w:num>
</w:numbering>""" % W_NS


def document_xml(body):
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="%s" xmlns:r="%s">
  <w:body>
    %s
    <w:sectPr>
      <w:pgSz w:w="12240" w:h="15840"/>
      <w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" w:header="708" w:footer="708" w:gutter="0"/>
      <w:cols w:space="720"/>
      <w:docGrid w:linePitch="360"/>
    </w:sectPr>
  </w:body>
</w:document>""" % (W_NS, R_NS, body)


def core_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
 xmlns:dc="http://purl.org/dc/elements/1.1/"
 xmlns:dcterms="http://purl.org/dc/terms/"
 xmlns:dcmitype="http://purl.org/dc/dcmitype/"
 xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>Python 上位机服务技术栈与系统架构</dc:title>
  <dc:creator>Codex</dc:creator>
  <cp:lastModifiedBy>Codex</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">2026-05-25T00:00:00Z</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">2026-05-25T00:00:00Z</dcterms:modified>
</cp:coreProperties>"""


def app_xml():
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties"
 xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Codex</Application>
  <DocSecurity>0</DocSecurity>
  <ScaleCrop>false</ScaleCrop>
</Properties>"""


def build():
    with open(MD_PATH, "r", encoding="utf-8") as handle:
        md_text = handle.read()
    body = parse_markdown(md_text)
    with zipfile.ZipFile(DOCX_PATH, "w", zipfile.ZIP_DEFLATED) as docx:
        docx.writestr("[Content_Types].xml", content_types_xml())
        docx.writestr("_rels/.rels", root_rels_xml())
        docx.writestr("word/_rels/document.xml.rels", document_rels_xml())
        docx.writestr("word/document.xml", document_xml(body))
        docx.writestr("word/styles.xml", styles_xml())
        docx.writestr("word/numbering.xml", numbering_xml())
        docx.writestr("docProps/core.xml", core_xml())
        docx.writestr("docProps/app.xml", app_xml())
    print(DOCX_PATH)


if __name__ == "__main__":
    build()
