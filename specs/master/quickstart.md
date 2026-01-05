# Quickstart Guide - Corrected

This guide provides the essential steps to get the "Physical AI & Humanoid Robotics" project up and running locally.

## Prerequisites

- **Node.js**: Version 18.x or higher
- **Python**: Version 3.11 or higher
- **Docker**: For running local instances of Postgres and Qdrant (optional, if not using cloud services)
- **Git**

## 1. Clone the Repository (if you haven't already)

```bash
git clone <repository-url>
cd <repository-directory>
```

## 2. Frontend Setup (Docusaurus)

**Important**: The Docusaurus project needs to be initialized. This step will create the Docusaurus application structure.

1.  **Initialize Docusaurus Project**:
    Navigate to the `physical-ai-book` directory and initialize the Docusaurus project. This will create a `docusaurus` sub-directory containing the application.

    ```bash
    cd physical-ai-book
    npm create docusaurus@latest docusaurus -- --typescript
    ```
    During the setup, choose the "classic" template.

2.  **Move Generated Content**:
    After initialization, Docusaurus will create a `docs` directory inside `physical-ai-book/docusaurus`. You will need to **delete** this newly generated `physical-ai-book/docusaurus/docs` directory and then **move** the `physical-ai-book/docs` directory (which contains all the modules and lessons we generated) into `physical-ai-book/docusaurus/`.

    ```bash
    # From the physical-ai-book directory:
    rmdir docusaurus\docs /s /q  # On Windows
    # rm -rf docusaurus/docs # On Linux/macOS

    mv docs docusaurus/docs   # On Windows (or cp -R for copy then rm)
    # mv docs docusaurus/docs # On Linux/macOS
    ```

3.  **Install Dependencies and Run**:
    Now, navigate into the created Docusaurus project directory, install dependencies, and start the development server.

    ```bash
    cd docusaurus

    # Install dependencies
    npm install

    # Start the local development server
    npm run start
    ```

    The book will be available at `http://localhost:3000`.

## 3. Backend Setup (FastAPI)

The backend is the FastAPI application that powers the RAG chatbot. (Note: The `backend` directory has not been created yet.)

```bash
# Navigate to the backend directory (once created)
cd backend

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows, use `venv\Scripts\activate`

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit the .env file with your credentials for Neon, Qdrant, and OpenAI
```

### Running the Backend

```bash
# Start the FastAPI server
uvicorn src.api.main:app --reload
```

The API will be available at `http://localhost:8000`, with documentation at `http://localhost:8000/docs`.

## 4. Running with Docker (Optional)

If you prefer to run the databases locally using Docker:

```bash
# In the backend directory
docker-compose up -d
```

This will start Postgres and Qdrant containers. Make sure your `.env` file points to these local instances.