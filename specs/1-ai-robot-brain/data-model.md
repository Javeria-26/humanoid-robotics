# Data Model: AI-Robot Brain Research Module

**Feature**: 1-ai-robot-brain
**Created**: 2025-12-18
**Status**: Draft

## Content Entities

### Module
- **Description**: The complete AI-Robot Brain Research Module
- **Fields**:
  - moduleId: unique identifier for the module
  - title: "Module 3 – The AI-Robot Brain (NVIDIA Isaac™)"
  - description: Overview of the module's purpose and content
  - targetAudience: ["Robotics students", "AI students", "Simulation practitioners"]
  - academicLevel: "Undergraduate to graduate level"
  - estimatedReadingTime: duration in minutes
  - learningObjectives: array of learning objectives
  - chapters: array of chapter references
  - references: reference list
  - createdAt: creation timestamp
  - updatedAt: last update timestamp

### Chapter
- **Description**: Individual chapters within the module
- **Fields**:
  - chapterId: unique identifier for the chapter
  - moduleId: reference to parent module
  - title: chapter title
  - subtitle: optional subtitle
  - chapterNumber: sequential number (1-4)
  - content: main content in Markdown format
  - learningObjectives: specific objectives for this chapter
  - keyConcepts: array of key concepts covered
  - references: chapter-specific references
  - wordCount: approximate word count
  - estimatedReadingTime: duration in minutes
  - createdAt: creation timestamp
  - updatedAt: last update timestamp

### Reference
- **Description**: Academic and technical references in APA format
- **Fields**:
  - referenceId: unique identifier for the reference
  - title: title of the source
  - authors: array of author names
  - publicationYear: year of publication
  - journal: name of journal/conference
  - volume: volume number (if applicable)
  - issue: issue number (if applicable)
  - pages: page range
  - doi: DOI identifier
  - url: URL to the source
  - type: ["Academic Paper", "Technical Documentation", "Book", "Conference Paper"]
  - isPeerReviewed: boolean indicating if peer-reviewed
  - citationText: full APA citation
  - summary: brief summary of content relevance

### LearningObjective
- **Description**: Specific learning objectives for the module and chapters
- **Fields**:
  - objectiveId: unique identifier for the objective
  - parentId: reference to module or chapter
  - description: detailed description of the objective
  - level: ["Knowledge", "Comprehension", "Application", "Analysis"]
  - measurable: boolean indicating if measurable
  - assessmentMethod: how the objective will be assessed

### KeyConcept
- **Description**: Important concepts covered in the module
- **Fields**:
  - conceptId: unique identifier for the concept
  - name: name of the concept
  - definition: clear definition of the concept
  - chapterId: reference to the chapter where it's covered
  - relatedConcepts: array of related concept IDs
  - examples: array of examples illustrating the concept
  - importance: ["Foundational", "Important", "Supplementary"]

## Relationships

### Module-Chapter Relationship
- One module contains many chapters (1:M)
- Each chapter belongs to exactly one module
- Module learning objectives may span multiple chapters

### Chapter-Reference Relationship
- One chapter may reference many sources (1:M)
- One reference may be used by multiple chapters (M:M through reference list)
- Each chapter must have at least 3 references

### Chapter-KeyConcept Relationship
- One chapter covers many key concepts (1:M)
- One key concept may appear in multiple chapters (M:M)
- Foundational concepts appear in earlier chapters

## Validation Rules

### Module Validation
- Title must be provided and not exceed 100 characters
- Description must be provided and between 50-300 characters
- Target audience must include at least one group
- Must contain exactly 4 chapters
- Learning objectives must be measurable
- Must include at least 15 references (with ≥50% peer-reviewed)

### Chapter Validation
- Title must be provided
- Content must be in valid Markdown format
- Chapter number must be between 1-4
- Estimated reading time must be between 15-60 minutes
- Must include at least 3 references
- Content must maintain academic tone
- Word count must be between 1000-2000 words

### Reference Validation
- Citation text must follow APA format
- At least 50% of references must be marked as peer-reviewed
- DOI or URL must be provided for verification
- Publication year must not be more than 10 years old (for technical sources)
- Type must be specified

### Learning Objective Validation
- Description must be specific and measurable
- Must align with target audience level
- Should connect to key concepts in the content

## State Transitions

### Chapter States
- Draft → In Review → Approved → Published
- Published → Needs Update → In Review → Approved → Published
- Each state transition requires specific validation checks

## Content Workflow

### Creation Workflow
1. Module is created with basic information
2. Chapters are created and linked to the module
3. Learning objectives are defined for the module and chapters
4. Key concepts are identified and defined
5. References are collected and validated
6. Content is written following academic standards
7. Content is reviewed and validated
8. Module is published to Docusaurus site

### Review Workflow
1. Peer review by technical expert
2. Academic review for tone and accuracy
3. Citation verification
4. Cross-reference validation
5. Quality assurance check
6. Approval for publication