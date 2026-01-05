# Data Model (Neon Postgres)

This document defines the SQL schema for the structured data stored in the serverless Neon Postgres database.

## Rationale

A relational database is required to store metadata that is not suitable for a vector database, such as user information, chat history, and relationships between them. Postgres is chosen for its reliability, feature set, and the serverless offering from Neon which fits the project's modern stack.

## Schema

### `users` table
Stores information about registered users.

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now())
);

-- Index for faster email lookups
CREATE INDEX idx_users_email ON users(email);
```

### `chats` table
Stores metadata for a single chat session, linking it to a user.

```sql
CREATE TABLE chats (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL,
    title VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),

    CONSTRAINT fk_user
        FOREIGN KEY(user_id) 
        REFERENCES users(id)
        ON DELETE CASCADE
);

-- Index for faster chat lookups by user
CREATE INDEX idx_chats_user_id ON chats(user_id);
```

### `message_logs` table
Logs every message in every chat, including the user's prompt and the AI's response.

```sql
CREATE TABLE message_logs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chat_id UUID NOT NULL,
    role VARCHAR(50) NOT NULL, -- 'user' or 'assistant'
    content TEXT NOT NULL,
    retrieved_context JSONB, -- Stores the context snippets from Qdrant
    created_at TIMESTAMP WITH TIME ZONE DEFAULT timezone('utc', now()),

    CONSTRAINT fk_chat
        FOREIGN KEY(chat_id) 
        REFERENCES chats(id)
        ON DELETE CASCADE
);

-- Index for faster message retrieval by chat
CREATE INDEX idx_message_logs_chat_id ON message_logs(chat_id);
```
