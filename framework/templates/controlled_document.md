<!--
doc_no: NASL-<CODE>-<TYPE>-<NNN>
doc_type: <DOCUMENT TYPE, e.g. TEST PROCEDURE>
title: <Project>: <title>
subtitle: <one line>
summary: <the cover summary: what the document is for and its main content, two or three sentences>
version: 1.0
date: <YYYY-MM-DD>
status: Draft
history: 1.0 | <YYYY-MM-DD> | First issue
prepared_by: <name or role>
reviewer: [TBD]
approver: [TBD]
entity: NASL Technologies (<unit>)
classification: Internal
footer: <Project> <short title>
file: NASL_<TYPE>-<NNN>_<Project>_<Short_Title>_v1.0
outdir: work/reports/documents
chapter_breaks: no
cover_image: none
request: <the request or the card this document answers, with the date>
scope: <what the document covers and what it does not>
related: <other documents by number>
-->
# <Title>

How to use this template (delete this paragraph): every controlled document starts with the header above. The header is an HTML comment, so a markdown viewer does not show it. Build the NASL Word file and PDF with `python tools/reports/build_document.py <this file>`. Never edit the generated PDF; change this source, raise the version, add a history line, and rebuild. Statuses: Draft, Issued for approval, Released, Superseded. Rules of style: NASL-EMS-HBK-001 section 8.6.

## Summary

<The result or the content in two or three sentences, then the key numbers.>

## 1. Purpose and scope

<Why the document exists; what it covers; what it does not.>

## 2. <Body chapters>

Table 1. <caption: tables are numbered by the builder in order>

| Column | Column |
|---|---|
| value | value |

![Figure 1: <caption, with what the axes show>](<path to .svg or .png from the project root>)

## References

- <documents by number and title; standards; files by path>
