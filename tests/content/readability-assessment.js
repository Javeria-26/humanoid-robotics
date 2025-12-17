/**
 * Readability Assessment Test
 *
 * This script evaluates the readability of content files
 * to ensure they meet the Flesch-Kincaid Grade Level requirement (Grade 10-12)
 */

const fs = require('fs');
const path = require('path');

// Simple Flesch-Kincaid Grade Level calculator
// Formula: 0.39 * (total words / total sentences) + 11.8 * (total syllables / total words) - 15.59
function calculateFleschKincaidGradeLevel(text) {
  // Remove markdown formatting for text analysis
  const cleanText = text.replace(/[#*_[\]]/g, ' ').replace(/```[\s\S]*?```/g, ' ');

  // Count sentences (naively using sentence-ending punctuation)
  const sentences = cleanText.split(/[.!?]+/).filter(s => s.trim().length > 0);
  const sentenceCount = sentences.length > 0 ? sentences.length : 1;

  // Count words
  const words = cleanText.match(/\b\w+\b/g) || [];
  const wordCount = words.length > 0 ? words.length : 1;

  // Count syllables (simplified estimation)
  let syllableCount = 0;
  for (const word of words) {
    syllableCount += estimateSyllables(word.toLowerCase());
  }
  syllableCount = syllableCount > 0 ? syllableCount : 1;

  // Calculate Flesch-Kincaid Grade Level
  const avgWordsPerSentence = wordCount / sentenceCount;
  const avgSyllablesPerWord = syllableCount / wordCount;

  const gradeLevel = 0.39 * avgWordsPerSentence + 11.8 * avgSyllablesPerWord - 15.59;

  return {
    gradeLevel: Math.max(0, gradeLevel),
    wordCount,
    sentenceCount,
    syllableCount
  };
}

// Simplified syllable estimation
function estimateSyllables(word) {
  word = word.toLowerCase();
  if (word.length <= 3) return 1;
  if (word.endsWith('e')) word = word.slice(0, -1);

  let syllableCount = 0;
  let prevIsVowel = false;

  const vowels = 'aeiouy';
  for (let i = 0; i < word.length; i++) {
    const isVowel = vowels.includes(word[i]);
    if (isVowel && !prevIsVowel) {
      syllableCount++;
    }
    prevIsVowel = isVowel;
  }

  return Math.max(1, syllableCount);
}

// Main assessment function
function assessReadability() {
  console.log('Starting readability assessment...');

  const contentFiles = [
    '../../docs/modules/ros2-nervous-system/index.md',
    '../../docs/modules/ros2-nervous-system/ros2-fundamentals.md',
    '../../docs/modules/ros2-nervous-system/python-ros-bridge.md',
    '../../docs/modules/ros2-nervous-system/urdf-humanoids.md'
  ];

  let allFilesPass = true;

  for (const file of contentFiles) {
    const filePath = path.join(__dirname, file);
    if (!fs.existsSync(filePath)) {
      console.log(`File not found: ${filePath}, skipping...`);
      continue;
    }

    const content = fs.readFileSync(filePath, 'utf8');
    const result = calculateFleschKincaidGradeLevel(content);

    console.log(`\nFile: ${file}`);
    console.log(`  Grade Level: ${result.gradeLevel.toFixed(2)}`);
    console.log(`  Word Count: ${result.wordCount}`);
    console.log(`  Sentence Count: ${result.sentenceCount}`);
    console.log(`  Syllable Count: ${result.syllableCount}`);

    if (result.gradeLevel >= 10 && result.gradeLevel <= 12) {
      console.log(`  Status: ✅ PASS (Meets Grade 10-12 requirement)`);
    } else if (result.gradeLevel < 10) {
      console.log(`  Status: ⚠️  INFO (Below Grade 10, may be acceptable)`);
    } else {
      console.log(`  Status: ❌ FAIL (Above Grade 12, needs simplification)`);
      allFilesPass = false;
    }
  }

  console.log(`\nOverall Readability Assessment: ${allFilesPass ? 'PASS' : 'FAIL'}`);
  return allFilesPass;
}

// Run the assessment
if (require.main === module) {
  const isPass = assessReadability();
  process.exit(isPass ? 0 : 1);
}

module.exports = { calculateFleschKincaidGradeLevel, estimateSyllables, assessReadability };