# Release review record policy

English | [日本語](README.md)

This directory is an audit archive preserving the findings and responses that led to a release decision. Current
user-facing documents in the parent directory are authoritative. Do not cite a historical review statement as
the current specification without checking later rounds.

## File names

```text
rNN-YYYY-MM-DD-findings.md
rNN-YYYY-MM-DD-response.md
```

- `rNN`: immutable two-digit review ID; do not renumber after publication
- `YYYY-MM-DD`: date on which the findings were finalized, in ISO 8601 format
- `findings`: findings, evidence, and decision at review time
- `response`: changes, verification, and remaining work in response to those findings

A date alone cannot distinguish multiple reviews on the same day, so the review ID is mandatory. A commit SHA
may change while a response is being prepared; keep SHAs in document metadata rather than the file name. Do not
use strings such as `#1` in file names because `#` has special meaning in URL fragments and shells.

## Required metadata

For R11 and later, findings and responses record the following where applicable:

- review ID
- review date
- package commit under review
- benchmark commit under review
- relative link to the corresponding findings or response
- decision: `Hold`, `Conditional Go`, or `Go`

Optional search tags such as `geometry`, `adapter`, `benchmark`, `provenance`, or `documentation` may be included
in the body. Tags supplement but do not replace the review ID.

R01–R10 preserve their original Japanese text as audit evidence. During migration, only IDs, file names, and
cross-links were normalized. In some records, target commits or the decision appear in the opening or conclusion
instead of dedicated metadata fields.

## Update rules

1. Keep findings and responses in separate files using the same review ID and date.
2. Link them in both directions and add one row to the [review history](../en/release_review_history.md).
3. If a later review changes a conclusion, do not rewrite the old record into the present tense. Explicitly
   withdraw or correct it in the later record.
4. Typographical errors and broken links may be fixed without changing meaning. A change to evidence or decision
   requires a new review record.
5. Prefer symbol names or heading anchors when referring to source. Do not use a mutable `#L123` link as the only
   evidence identifier; include the target commit when needed.
6. After moving or renaming records, validate relative links across all Markdown files, including READMEs.
7. The package commit named in the history's `Current state` cannot be written by the response commit itself - a
   commit cannot contain its own SHA. **Always follow the response commit with a small commit that records that
   SHA.** R12 L4, R13 L5 and R14 M10 are the same defect recurring because this step did not exist.

## Current records

See the [release review history](../en/release_review_history.md) for the per-round summary, final decision, and
withdrawn statements. If an individual record conflicts with the history, the later review takes precedence.
