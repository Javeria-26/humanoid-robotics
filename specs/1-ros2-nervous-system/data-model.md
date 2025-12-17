# Data Model: ROS 2 Educational Module

**Feature**: 1-ros2-nervous-system
**Date**: 2025-12-16

## Content Entities

### Chapter
- **name**: String (e.g., "ROS 2 Fundamentals", "Python-ROS Bridge", "URDF for Humanoids")
- **description**: String (brief overview of chapter content)
- **learning_objectives**: Array of strings (specific skills/knowledge students will gain)
- **sections**: Array of Section entities
- **diagrams**: Array of Diagram entities
- **code_examples**: Array of CodeExample entities
- **references**: Array of Reference entities

### Section
- **title**: String (section heading)
- **content**: String (main content in Markdown format)
- **order**: Integer (sequence within chapter)
- **prerequisites**: Array of strings (knowledge required before this section)
- **learning_outcomes**: Array of strings (what student should understand after reading)

### Diagram
- **title**: String (diagram name/description)
- **filename**: String (reference to image file in docs/diagrams/)
- **description**: String (explanation of what the diagram illustrates)
- **alt_text**: String (accessibility text for screen readers)

### CodeExample
- **title**: String (brief description of the example)
- **filename**: String (reference to code file in src/components/code-examples/)
- **language**: String (programming language, e.g., "python")
- **description**: String (explanation of what the code demonstrates)
- **usage_context**: String (where in the text this example is referenced)

### Reference
- **citation**: String (full APA format citation)
- **type**: String (e.g., "academic_paper", "official_documentation", "book", "website")
- **access_date**: Date (when the source was accessed)
- **url**: String (URL if applicable)
- **relevance**: String (how this source relates to the module content)

## Content Relationships

- Chapter contains multiple Sections
- Chapter references multiple Diagrams
- Chapter includes multiple CodeExamples
- Chapter contains multiple References
- Sections may reference other Sections for cross-referencing
- CodeExamples are associated with specific Sections where they are discussed

## Validation Rules

- Each Chapter must have a unique name within the module
- Each Section must belong to exactly one Chapter
- Learning objectives must be specific and measurable
- All external references must be properly cited in APA format
- Code examples must be tested and verified to work
- Diagrams must have appropriate alt text for accessibility
- All content must meet Flesch-Kincaid Grade 10-12 readability standards

## State Transitions (if applicable)

For content review workflow:
- DRAFT → UNDER_REVIEW (when content is ready for review)
- UNDER_REVIEW → NEEDS_REVISION (when feedback requires changes)
- UNDER_REVIEW → APPROVED (when content passes review)
- NEEDS_REVISION → UNDER_REVIEW (after revisions are made)
- APPROVED → PUBLISHED (when module is ready for deployment)