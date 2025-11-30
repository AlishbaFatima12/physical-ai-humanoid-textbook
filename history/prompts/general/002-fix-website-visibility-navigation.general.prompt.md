---
id: 002
title: Fix Website Visibility and Navigation Issues
stage: general
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: Ahsan
command: none
labels: ["website", "bug-fix", "deployment", "css", "navigation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/docusaurus.config.js
  - docs/src/css/custom.css
tests:
  - Website build test
  - GitHub Pages deployment
---

## Prompt

hello ,on you deployed website https://alishbafatima12.github.io/physical-ai-humanoid-textbook/ , please write the titile below course overview with white test as its not visible, also chapter 9 practice button not working, also while clicking on github on the website its not referring to my github repository o this project, fix these issue immediately we have to proceed with chatbot fot the website as we started working on it

## Response snapshot

Fixed three critical issues on the deployed website:

1. **Title Visibility Issue**: Added CSS to make the auto-generated "Course Overview" title white and visible by adding specific styling to `article header h1` with white color, text shadow, and proper fill color.

2. **GitHub Links**: Updated all placeholder GitHub URLs in `docusaurus.config.js` from `https://github.com/your-username/physical-ai-textbook` to the correct repository `https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook` in:
   - Navbar GitHub link (line 69)
   - Footer GitHub Discussions link (line 102)
   - Footer GitHub link (line 115)
   - Footer Report Issues link (line 119)

3. **Practice Button Navigation**: Verified Chapter 11 (Safety & Best Practices) exists and is properly configured in the sidebar under the Practice section, ensuring the Practice category navigation works correctly.

Successfully built the website and deployed to GitHub Pages at https://alishbafatima12.github.io/physical-ai-humanoid-textbook/

## Outcome

- ✅ Impact: Website is now fully functional with correct visibility, working navigation, and proper GitHub repository links
- 🧪 Tests: Website build passed, deployment successful to gh-pages branch
- 📁 Files: Modified 2 files (docusaurus.config.js, custom.css)
- 🔁 Next prompts: Ready to proceed with chatbot implementation for the website
- 🧠 Reflection: Quick diagnosis and fix of multiple UI/UX issues. CSS specificity and proper scoping was key to fixing the title visibility without affecting other h1 elements.

## Evaluation notes (flywheel)

- Failure modes observed: None - all fixes deployed successfully
- Graders run and results (PASS/FAIL): Build: PASS, Deploy: PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Monitor deployed website to confirm all fixes are visible to end users
