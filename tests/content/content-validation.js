/**
 * Content Validation Test
 *
 * This script validates the content quality, structure, and consistency
 * across all documentation files
 */

const fs = require('fs');
const path = require('path');

// Function to validate markdown structure
function validateMarkdownStructure(content, filePath) {
  const errors = [];

  // Check for proper frontmatter
  if (!content.includes('---')) {
    errors.push('Missing frontmatter delimiters (---)');
  }

  // Check for proper headings hierarchy
  const lines = content.split('\n');
  let hasH1 = false;
  for (const line of lines) {
    if (line.trim().startsWith('# ')) {
      hasH1 = true;
      break;
    }
  }

  if (!hasH1) {
    errors.push('No H1 heading found');
  }

  // Check for common issues
  if (content.includes('TODO') || content.includes('FIXME')) {
    errors.push('Contains TODO or FIXME markers');
  }

  return errors;
}

// Function to validate internal links
function validateInternalLinks(content, currentFile) {
  const errors = [];

  // Find all markdown links
  const linkPattern = /\[([^\]]+)\]\(([^)]+)\)/g;
  let match;

  while ((match = linkPattern.exec(content)) !== null) {
    const linkText = match[1];
    const linkUrl = match[2];

    // Check if it's an internal link (doesn't start with http or #)
    if (!linkUrl.startsWith('http') && !linkUrl.startsWith('#')) {
      // Resolve relative path
      const resolvedPath = path.resolve(path.dirname(currentFile), linkUrl);

      // For this validation, we just check if it looks like a valid path
      if (linkUrl.endsWith('.md') && !fs.existsSync(resolvedPath)) {
        errors.push(`Internal link points to non-existent file: ${linkUrl}`);
      }
    }
  }

  return errors;
}

// Main validation function
function validateContent() {
  console.log('Starting content validation...');

  const contentFiles = [
    '../../docs/index.md',
    '../../docs/modules/ros2-nervous-system/index.md',
    '../../docs/modules/ros2-nervous-system/ros2-fundamentals.md',
    '../../docs/modules/ros2-nervous-system/python-ros-bridge.md',
    '../../docs/modules/ros2-nervous-system/urdf-humanoids.md'
  ];

  let allValid = true;

  for (const file of contentFiles) {
    const filePath = path.join(__dirname, file);
    if (!fs.existsSync(filePath)) {
      console.log(`File not found: ${filePath}, skipping...`);
      continue;
    }

    console.log(`\nValidating: ${file}`);
    const content = fs.readFileSync(filePath, 'utf8');

    // Validate structure
    const structureErrors = validateMarkdownStructure(content, filePath);
    if (structureErrors.length > 0) {
      console.log(`  ❌ Structure issues: ${structureErrors.join(', ')}`);
      allValid = false;
    } else {
      console.log('  ✅ Structure validation passed');
    }

    // Validate internal links
    const linkErrors = validateInternalLinks(content, filePath);
    if (linkErrors.length > 0) {
      console.log(`  ❌ Link issues: ${linkErrors.join(', ')}`);
      allValid = false;
    } else {
      console.log('  ✅ Link validation passed');
    }
  }

  console.log(`\nOverall Content Validation: ${allValid ? 'PASS' : 'FAIL'}`);
  return allValid;
}

// Run the validation
if (require.main === module) {
  const isValid = validateContent();
  process.exit(isValid ? 0 : 1);
}

module.exports = { validateMarkdownStructure, validateInternalLinks, validateContent };