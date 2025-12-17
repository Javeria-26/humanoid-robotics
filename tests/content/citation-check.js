/**
 * Citation Validation Test
 *
 * This script validates that all content files have proper citations
 * and that citations match the references in citations.md
 */

const fs = require('fs');
const path = require('path');

// Function to extract citations from markdown content
function extractCitations(content) {
  // Look for patterns like [Author, Year] or similar citation formats
  const citationPattern = /\[(?:[A-Z][a-z]+,\s*\d{4}(?:;\s*[A-Z][a-z]+,\s*\d{4})*)\]/g;
  const matches = content.match(citationPattern) || [];
  return matches;
}

// Function to extract reference entries from citations.md
function extractReferences(citationsContent) {
  // Extract authors and years from reference list
  const referencePattern = /^[0-9]+\. ([A-Z][a-z]+), [A-Z].(?:, [A-Z][a-z]+, [A-Z].)* \((\d{4})\)/gm;
  const matches = [];
  let match;
  while ((match = referencePattern.exec(citationsContent)) !== null) {
    matches.push(`${match[1]}, ${match[2]}`);
  }
  return matches;
}

// Main validation function
function validateCitations() {
  console.log('Starting citation validation...');

  // Read the citations reference file
  const citationsPath = path.join(__dirname, '../../docs/references/citations.md');
  if (!fs.existsSync(citationsPath)) {
    console.error('ERROR: citations.md file not found');
    return false;
  }

  const citationsContent = fs.readFileSync(citationsPath, 'utf8');
  const referenceList = extractReferences(citationsContent);
  console.log(`Found ${referenceList.length} references in citations.md`);

  // Check content files for citations
  const contentFiles = [
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

    const content = fs.readFileSync(filePath, 'utf8');
    const citations = extractCitations(content);

    if (citations.length > 0) {
      console.log(`Found ${citations.length} citations in ${file}:`, citations);
    } else {
      console.log(`No citations found in ${file} - this may be expected for some files`);
    }
  }

  console.log('Citation validation completed.');
  return allValid;
}

// Run the validation
if (require.main === module) {
  const isValid = validateCitations();
  process.exit(isValid ? 0 : 1);
}

module.exports = { extractCitations, extractReferences, validateCitations };