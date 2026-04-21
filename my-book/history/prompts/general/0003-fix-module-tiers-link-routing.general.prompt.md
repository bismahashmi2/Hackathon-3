---
id: 0003
title: "Fix Module Tiers Link Routing"
stage: "general"
date: "2026-01-22"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "main"
user: "tell_hashmi-hashmi"
command: "fix-broken-links"
labels: ["bugfix", "routing", "ui", "module-tiers"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "src/components/ModuleTiers/index.tsx"
tests: []
---

## Prompt

User reported that the link on line 159 in the ModuleTiers component was broken due to a wildcard character in the path. The sidebar is auto-generated, and the user wanted the links to properly route to the correct module theory pages.

## Response snapshot

Identified the issue where the original code used a wildcard pattern `/docs/textbook/modules/${module.number.padStart(2, '0')}-*/theory` which would not resolve to actual document paths. Examined the filesystem structure and found that each module has a directory with the pattern `{number}-{title-in-kebab-case}` containing a `theory.md` file.

Created a mapping function `getModuleDirectoryName()` that properly converts module titles to the correct directory names that match the actual filesystem structure. The function handles special cases where the directory naming doesn't follow a simple pattern (like removing "and" from some titles but keeping "to" in others).

Updated the Link component to use the new mapping function instead of the wildcard pattern. Handled special cases for inconsistent naming patterns.

Verified that all module directories exist with the expected `theory.md` files, ensuring the links will work correctly.

## Outcome

- ✅ Impact: Fixed broken links in ModuleTiers component allowing proper navigation to module theory pages
- 🧪 Tests: No tests required as this is a UI routing fix
- 📁 Files: Modified src/components/ModuleTiers/index.tsx to implement proper link routing
- 🔁 Next prompts: None needed, issue resolved
- 🧠 Reflection: The directory naming pattern wasn't consistent, requiring explicit mappings for some modules

## Evaluation notes (flywheel)

- Failure modes observed: Wildcard pattern in URL paths causes broken navigation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A