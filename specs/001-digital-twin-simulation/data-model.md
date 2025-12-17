# Data Model: Digital Twin Simulation Module

## Educational Content Entities

### Module
- **Name**: String (required) - The module title
- **Description**: Text (required) - Overview of the module content
- **TargetAudience**: Array of strings (required) - ["students", "educators"] or similar
- **LearningObjectives**: Array of text (required) - Specific learning goals
- **Duration**: Integer (optional) - Estimated time to complete in minutes
- **Prerequisites**: Array of text (optional) - Knowledge required before starting

### Chapter
- **Title**: String (required) - The chapter title
- **Content**: Text (required) - The main content of the chapter
- **ModuleId**: String (required) - Reference to parent module
- **Order**: Integer (required) - Chapter sequence number
- **LearningOutcomes**: Array of text (required) - What learners will understand
- **KeyTerms**: Array of strings (optional) - Important terminology

### ResearchSource
- **Title**: String (required) - Title of the source
- **Authors**: Array of strings (required) - Author names
- **Year**: Integer (required) - Publication year
- **Journal**: String (optional) - Publication venue
- **DOI**: String (optional) - Digital object identifier
- **CitationType**: String (required) - ["peer-reviewed", "conference", "book", "web"]
- **Relevance**: Text (optional) - How the source relates to the content

### SensorType
- **Name**: String (required) - ["LiDAR", "depth-camera", "IMU", etc.]
- **Characteristics**: Text (required) - Key properties and capabilities
- **Limitations**: Text (required) - Known limitations and constraints
- **SimulationModel**: Text (required) - How it's simulated in tools
- **UseCases**: Array of text (required) - Typical applications

### SimulationTool
- **Name**: String (required) - ["Gazebo", "Unity"]
- **PrimaryUse**: String (required) - ["physics-simulation", "visualization"]
- **Capabilities**: Array of text (required) - What the tool can do
- **Limitations**: Array of text (required) - Known constraints
- **BestPractices**: Array of text (optional) - Recommended usage patterns

## Relationships
- Module **has many** Chapters
- Chapter **references many** ResearchSources
- Module **utilizes many** SimulationTools
- Chapter **covers many** SensorTypes

## Validation Rules
- Module.title must be 5-100 characters
- Module.learningObjectives must have at least 1 item
- Chapter.title must be 5-100 characters
- Chapter.learningOutcomes must have at least 1 item
- ResearchSource.year must be between 2015-2025
- ResearchSource.citationType must be one of the allowed values