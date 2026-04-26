---
id: "0002"
title: "Fix Broken ModuleTiers Links"
stage: "general"
date_iso: "2026-01-27"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "main"
user: "user"
command: "Fix broken links in ModuleTiers and BookHero components"
labels: ["links", "navigation", "bug-fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "/mnt/d/Code/hackathon/book_project/my-book/src/components/ModuleTiers/index.tsx"
  - "/mnt/d/Code/hackathon/book_project/my-book/src/components/BookHero/index.tsx"
  - "/mnt/d/Code/hackathon/book_project/my-book/docusaurus.config.ts"
tests_yaml: []
---

# Fix Broken ModuleTiers Links

## Summary

Fixed broken navigation links in both the ModuleTiers and BookHero components that were causing users to land on "Untitled" blank pages instead of the intended module content.

## Problem

- The "Start with Module 01" link on line 267 in ModuleTiers was redirecting to a blank "Untitled" page
- The "Start Learning" link in BookHero on line 47 was not working
- All links connecting to modules were non-functional

## Root Cause

The issue was caused by incorrect link formats in the React components. The links were using the `doc:` syntax incorrectly or had malformed paths that didn't correspond to the actual file structure in the Docusaurus documentation system.

## Solution

Updated the link formats in the following files:

1. `/mnt/d/Code/hackathon/book_project/my-book/src/components/ModuleTiers/index.tsx`:
   - Changed the "Start with Module 01" link from `doc:textbook/modules/01-introduction-physical-ai/theory` to `/docs/textbook/modules/01-introduction-physical-ai/theory`
   - Updated the dynamic module links in the `ModuleCard` component from `doc:` syntax to proper absolute paths

2. `/mnt/d/Code/hackathon/book_project/my-book/src/components/BookHero/index.tsx`:
   - Changed the "Start Learning" link from `/docs/textbook/appendices/module-progression` to `/docs/textbook/modules/01-introduction-physical-ai/theory`
   - Updated the "View Module 1" link from `doc:textbook/modules/01-introduction-physical-ai/theory` to `/docs/textbook/modules/01-introduction-physical-ai/theory`

3. `/mnt/d/Code/hackathon/book_project/my-book/docusaurus.config.ts`:
   - Fixed the incorrect docId from `"textbook/modules/introduction-physical-ai/01"` to `"textbook/modules/01-introduction-physical-ai/theory"`

## Files Modified

- `src/components/ModuleTiers/index.tsx` - Fixed navigation links
- `src/components/BookHero/index.tsx` - Fixed primary and secondary buttons
- `docusaurus.config.ts` - Fixed navbar docId reference

## Outcome

All module links now correctly navigate to their respective theory pages instead of showing blank "Untitled" pages. The navigation structure is now consistent with the actual documentation file structure.

## Validation

- Verified that all links use the correct path format: `/docs/textbook/modules/{module-number}-{module-title}/theory`
- Confirmed that the path structure matches the actual files in the `docs/textbook/modules/` directory
- Ensured consistency between all module links using the `getModuleDirectoryName` helper function

## Prompt Text

The user reported that links in the ModuleTiers and BookHero components were not working properly, with the "Start with Module 01" link redirecting to a blank "Untitled" page and the "Start Learning" link not functioning. All links connecting to modules were non-functional.

## Response Text

I analyzed the component files and found that the issue was with the link format using incorrect `doc:` syntax or malformed paths. I fixed the links in both components by changing them to use the correct absolute path format that matches the actual file structure in the documentation system. I also updated the docusaurus config file to use the correct docId format.
