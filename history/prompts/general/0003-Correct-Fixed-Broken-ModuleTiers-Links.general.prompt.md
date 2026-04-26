---
id: "0003"
title: "Correct Fixed Broken ModuleTiers Links"
stage: "general"
date_iso: "2026-01-27"
surface: "agent"
model: "claude-opus-4-5-20251101"
feature: "none"
branch: "main"
user: "user"
command: "Corrected the fixes for broken links in ModuleTiers and BookHero components"
labels: ["links", "navigation", "bug-fix", "correction"]
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

# Correct Fixed Broken ModuleTiers Links

## Summary

Corrected the previous fix for broken navigation links after discovering that the document IDs in Docusaurus follow a different format than initially assumed.

## Previous Problem

- The "Start with Module 01" link on line 267 in ModuleTiers was redirecting to a blank "Untitled" page
- The "Start Learning" link in BookHero on line 47 was not working
- All links connecting to modules were non-functional

## Previous Solution (Incorrect)

I initially changed all links to use absolute paths like `/docs/textbook/modules/01-introduction-physical-ai/theory`, but this was incorrect based on the Docusaurus auto-generated sidebar structure.

## Issue Discovered

After attempting to build the site, I received an error indicating that the correct document ID format is `textbook/modules/{module-name}/{module-number}` rather than the path-based format I was using. For example:
- Correct: `textbook/modules/introduction-physical-ai/01`
- Incorrect: `textbook/modules/01-introduction-physical-ai/theory`

## Correction Applied

Reverted the changes and implemented the correct document ID format:

1. `/mnt/d/Code/hackathon/book_project/my-book/docusaurus.config.ts`:
   - Changed docId back to `"textbook/modules/introduction-physical-ai/01"`

2. `/mnt/d/Code/hackathon/book_project/my-book/src/components/BookHero/index.tsx`:
   - Changed links to use `doc:textbook/modules/introduction-physical-ai/01` format

3. `/mnt/d/Code/hackathon/book_project/my-book/src/components/ModuleTiers/index.tsx`:
   - Added a new `getModuleDocId()` function to generate correct document IDs following the format `{module-name}/{module-number}`
   - Updated all module links to use the `doc:` syntax with the correct ID format
   - Created explicit mappings for all 14 modules to ensure correct document IDs

## Files Modified

- `src/components/ModuleTiers/index.tsx` - Added `getModuleDocId` function and updated all links
- `src/components/BookHero/index.tsx` - Fixed navigation links to use correct doc ID format
- `docusaurus.config.ts` - Reverted to correct docId format

## Outcome

All module links now use the correct document ID format that matches the auto-generated sidebar structure, which should resolve the navigation issues while maintaining compatibility with the Docusaurus system.

## Validation

- Document IDs now follow the format `textbook/modules/{module-name}/{module-number}` as required by the auto-generated sidebar
- Created explicit mappings for all 14 modules to ensure accurate document references
- Maintained the `doc:` syntax which is the proper way to reference documents in Docusaurus

## Prompt Text

The user reported that after my initial fixes, the site was crashing with an error about not finding documents with the IDs I had used. The error showed that the correct document ID format was `textbook/modules/{module-name}/{module-number}` based on the auto-generated sidebar structure.

## Response Text

I analyzed the error message and corrected my previous changes to use the proper document ID format that matches the auto-generated sidebar structure. I created a new helper function to generate the correct document IDs for all modules and updated all links to use the proper `doc:` syntax with the correct ID format.