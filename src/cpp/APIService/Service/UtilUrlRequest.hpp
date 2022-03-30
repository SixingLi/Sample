// Copy from https://github.com/bakwc/htf/
#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>
#include <algorithm>
#ifdef __unix__
#define __nixbuild__
#endif
#ifdef __APPLE__
#define __nixbuild__
#endif

#ifdef _WIN32
#define _WINSOCKAPI_
#include <winsock2.h>
#endif


#ifdef __nixbuild__
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
typedef int SOCKET;
typedef sockaddr_in SOCKADDR_IN;
typedef sockaddr SOCKADDR;
#endif

using namespace std;

namespace UtilUrlRequest {
	
	enum Mode
	{
		GET = 1,
		POST
	};

	struct TAddr;
	struct TResult;
	typedef unordered_map<string, string> THeaders;

	// Downloads HTTP url from web, eg.
	// string data = GetUrl("http://example.com");
	// throws exception in case of error
	inline string GetUrl(const string& url,
		const THeaders& headers = THeaders(),
		size_t timeout = 30);

	inline string GetUrlEx(const string& url,
		const THeaders& headers = THeaders(),
		size_t timeout = 30);

	inline string GetUrl(const string& url, size_t timeout) {
		return GetUrl(url, THeaders(), timeout);
	}

	inline string PostUrl(const string& url,
		const std::string& postData,
		const THeaders& headers = THeaders(),
		size_t timeout = 30);

	// Same, but no exceptions - returns struct with result
	inline TResult Fetch(Mode mode,
		const string& url,
		const THeaders& headers = THeaders(),
		const std::string& postData = "",
		size_t timeout = 30);

	inline TResult FetchEx(Mode mode,
		const string& url,
		const THeaders& headers = THeaders(),
		const std::string& postData = "",
		size_t timeout = 30);

	inline TAddr ParseUrl(string url);
	inline void Clean(string& str);
	inline void Split(const string& str, string& str1, string& str2);
	inline size_t stoh(const string& str);

	struct TAddr {
		string Protocol;
		string Host;
		unsigned short Port;
		string Action;
	};

	enum EResultType {
		Success,
		ConnectionTimeout,
		SocketTimeout,
		SocketError,
		HostNotFound,
		ResolveError,
		WrongUrl,
		HttpError,
		InitializationError
	};

	struct TResult {
		EResultType Type;
		size_t Code;
		THeaders Headers;
		string ResolvedUrl; // In case of redirects may not be the same as requested
		string Content;
	};

	inline string ResultTypeToStr(EResultType result) {
		switch (result) {
		case Success: return "Success"; break;
		case ConnectionTimeout: return "ConnectionTimeout"; break;
		case SocketTimeout: return "SocketTimeout"; break;
		case SocketError: return "SocketError"; break;
		case HostNotFound: return "HostNotFound"; break;
		case ResolveError: return "ResolveError"; break;
		case WrongUrl: return "WrongUrl"; break;
		case HttpError: return "HttpError"; break;
		case InitializationError: return "InitializationError"; break;
		default: return "Unknown"; break;
		}
	}

	class TException : public exception {
	public:
		TException(EResultType type, int code = -1)
			: Type(type)
			, TypeStr(ResultTypeToStr(type))
			, Code(code)
		{
		}
		const char* what() const noexcept(true) {
			return TypeStr.c_str();
		}
	private:
		EResultType Type;
		string TypeStr;
		int Code;
	};

	inline string PostUrl(const string& url,
		const std::string& postData,
		const THeaders& headers,
		size_t timeout)
	{
		TResult res = Fetch(Mode::POST, url, headers, postData, timeout);
		if (res.Type != Success) {
			throw TException(res.Type, static_cast<int>(res.Code));
		}
		return res.Content;
	}

	inline string GetUrl(const string& url, const THeaders& headers, size_t timeout) {
		TResult res = Fetch(Mode::GET, url, headers, "", timeout);
		if (res.Type != Success) {
			throw TException(res.Type, static_cast<int>(res.Code));
		}
		return res.Content;
	}

	inline string GetUrlEx(const string& url, const THeaders& headers, size_t timeout) {
		TResult res = FetchEx(Mode::GET, url, headers, "", timeout);
		if (res.Type != Success) {
			throw TException(res.Type, static_cast<int>(res.Code));
		}
		return res.Content;
	}

	inline TResult Fetch(Mode mode, const string& url, const THeaders& headers, const std::string& postData, size_t timeout) {
#ifdef _WIN32
		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			TResult res;
			res.Code = -1;
			res.Type = InitializationError;
			return res;
		}
#endif
		TAddr addr = ParseUrl(url);
		SOCKET Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		struct hostent *host;
#pragma   warning(push) 
#pragma   warning(disable:4996)  
		// warning C4996: 'gethostbyname': Use getaddrinfo() or GetAddrInfoW() instead or define _WINSOCK_DEPRECATED_NO_WARNINGS to disable deprecated API warnings
		host = gethostbyname(addr.Host.c_str()); 
#pragma   warning(pop)
		if (host == NULL) {
			TResult res;
			res.Code = -1;
			res.Type = ResolveError;
			if (h_errno == HOST_NOT_FOUND) {
				res.Type = HostNotFound;
			}
			return res;
		}
		SOCKADDR_IN SockAddr;
		SockAddr.sin_port = htons(addr.Port);
		SockAddr.sin_family = AF_INET;
		SockAddr.sin_addr.s_addr = *((unsigned long*)host->h_addr);
		if (connect(Socket, (SOCKADDR*)(&SockAddr), sizeof(SockAddr)) != 0) {
			TResult res;
			res.Code = -1;
			res.Type = ConnectionTimeout;
			return res;
		}
		string request;
		switch (mode)
		{
		case UtilUrlRequest::GET:
			request = "GET " + addr.Action + " HTTP/1.1\r\n";
			break;
		case UtilUrlRequest::POST:
			request = "POST " + addr.Action + " HTTP/1.1\r\n";
			break;
		}

		if (headers.find("Host") == headers.end())
		{
			request += "Host: " + addr.Host + "\r\n";
		}
		if (mode == UtilUrlRequest::POST)
		{
			request += "Content-Length: " + std::to_string(postData.length()) + "\r\n";
		}

		for (auto &it : headers) {
			request += it.first + ": " + it.second + "\r\n";
		}
		request += "Connection: close\r\n\r\n";
		if (mode == UtilUrlRequest::POST)
		{
			request += postData;
		}
		if (send(Socket, request.c_str(), static_cast<int>(request.length()), 0) == -1) {
			TResult res;
			res.Code = -1;
			res.Type = SocketError;
			return res;
		}
		TResult res;

		//char buffer[2000] = {0};
		//int count = recv(Socket, buffer, 2000, 0);

		//char* buffer = new char[1024 * 1024 * 4];
		std::vector<char>content;
		int chunk = 1024 * 101;
		
		int count = 0;
		int repeat = 1;
		while (true)
		{
			content.resize(repeat*chunk);
			int tmp = recv(Socket, &content[count], 1024 * 100, 0);
			count += tmp;
			if (tmp <= 0) {
				break;
			}
			repeat += 1;
		}
		char* buffer = &content[0];
		if (buffer == NULL) {
			res.Code = -1;
			res.Type = HttpError;
			return res;
		}
		string firstData(buffer, count);
		size_t headerDelimPos = firstData.find("\r\n\r\n");
		if (headerDelimPos == string::npos) {
			res.Code = -1;
			res.Type = HttpError;
			return res;
		}
		size_t start, delim = string::npos;

		size_t i = 0;
		size_t spacesCount = 0;

		for (; buffer[i] != '\r' && buffer[i + 1] != '\n'; i++) {
			if (buffer[i] == ' ') {
				spacesCount++;
				if (spacesCount == 2) {
					res.Code = stoi(string(buffer + start, i - start));
				}
				start = i + 1;
			}
		}

		if (res.Code != 200) {
			res.Type = HttpError;
			return res;
		}

		i += 2;
		start = i;

		for (; i < headerDelimPos + 1; i++) {
			if (buffer[i] == ':' && delim == string::npos) {
				delim = i;
			}
			else if (buffer[i] == '\r' && buffer[i + 1] == '\n') {
				if (delim == string::npos) {
					res.Code = -1;
					res.Type = HttpError;
					return res;
				}
				std::string headerKey = string(buffer + start, delim - start);
				std::string temp = headerKey;
				std::transform(temp.begin(), temp.end(), temp.begin(), ::toupper);
				std::string headerValue = string(buffer + delim + 2, i - delim - 2);
				if (temp.find("CONTENT-LENGTH") != std::string::npos)
				{
					res.Headers["CONTENT-LENGTH"] = headerValue;
				}
				else
				{
					res.Headers[headerKey] = headerValue;
				}
				delim = string::npos;
				start = i + 2;
			}
		}
		res.Content += string(buffer + i + 3, count - i - 3);

		// res.Headers
		THeaders::iterator headIt = res.Headers.find("CONTENT-LENGTH");
		size_t receivedCount = res.Content.length();
		size_t totalCount = receivedCount;
		if (headIt != res.Headers.end())
		{
			totalCount = std::atol(headIt->second.c_str());
		}

		while (receivedCount < totalCount) {
			count = recv(Socket, buffer, 1000, 0);
			receivedCount += count;
			res.Content += string(buffer, count);
		}

		if (res.Headers.find("Transfer-Encoding") != res.Headers.end() &&
			res.Headers["Transfer-Encoding"] == "chunked")
		{
			string result;
			size_t i = 0, start = 0;
			for (; i < res.Content.size(); i++) {
				if (res.Content[i] == '\r' && res.Content[i + 1] == '\n') {
					string chunkSizeStr = res.Content.substr(start, i - start);
					size_t chunkSize = stoh(chunkSizeStr);
					result += res.Content.substr(i + 2, chunkSize);
					i += chunkSize + 2;
					start = i + 2;
				}
			}
			res.Content = result;
		}

#ifdef _WIN32
		closesocket(Socket);
		WSACleanup();
#endif
#ifdef __unix__
		shutdown(Socket, SHUT_RDWR);
		close(Socket);
#endif
		res.Type = Success;

		return res;
	}


	inline TResult FetchEx(Mode mode, const string& url, const THeaders& headers, const std::string& postData, size_t timeout) {
#ifdef _WIN32
		WSADATA wsaData;
		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			TResult res;
			res.Code = -1;
			res.Type = InitializationError;
			return res;
		}
#endif
		TAddr addr = ParseUrl(url);
		SOCKET Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		struct hostent *host;
#pragma   warning(push) 
#pragma   warning(disable:4996)  
		// warning C4996: 'gethostbyname': Use getaddrinfo() or GetAddrInfoW() instead or define _WINSOCK_DEPRECATED_NO_WARNINGS to disable deprecated API warnings
		host = gethostbyname(addr.Host.c_str());
#pragma   warning(pop)
		if (host == NULL) {
			TResult res;
			res.Code = -1;
			res.Type = ResolveError;
			if (h_errno == HOST_NOT_FOUND) {
				res.Type = HostNotFound;
			}
			return res;
		}
		SOCKADDR_IN SockAddr;
		SockAddr.sin_port = htons(addr.Port);
		SockAddr.sin_family = AF_INET;
		SockAddr.sin_addr.s_addr = *((unsigned long*)host->h_addr);
		if (connect(Socket, (SOCKADDR*)(&SockAddr), sizeof(SockAddr)) != 0) {
			TResult res;
			res.Code = -1;
			res.Type = ConnectionTimeout;
			return res;
		}
		string request;
		switch (mode)
		{
		case UtilUrlRequest::GET:
			request = "GET " + addr.Action + " HTTP/1.1\r\n";
			break;
		case UtilUrlRequest::POST:
			request = "POST " + addr.Action + " HTTP/1.1\r\n";
			break;
		}

		if (headers.find("Host") == headers.end())
		{
			request += "Host: " + addr.Host + "\r\n";
		}
		if (mode == UtilUrlRequest::POST)
		{
			request += "Content-Length: " + std::to_string(postData.length()) + "\r\n";
		}

		for (auto &it : headers) {
			request += it.first + ": " + it.second + "\r\n";
		}
		request += "Connection: close\r\n\r\n";
		if (mode == UtilUrlRequest::POST)
		{
			request += postData;
		}
		if (send(Socket, request.c_str(), static_cast<int>(request.length()), 0) == -1) {
			TResult res;
			res.Code = -1;
			res.Type = SocketError;
			return res;
		}
		TResult res;
		string content;
		while (true)
		{
			int max_length = 1024 * 10;
			char buffer[1024*10] = { 0 };
			int count = recv(Socket, buffer, max_length, 0);
			if (count > 0) {
				content += string(buffer, count);
				continue;
			}
			break;
			
		}

		size_t headerDelimPos = content.find("\r\n\r\n");
		if (headerDelimPos == string::npos) {
			res.Code = -1;
			res.Type = HttpError;
			return res;
		}
		size_t start, delim = string::npos;

		size_t i = 0;
		size_t spacesCount = 0;

		for (; content[i] != '\r' && content[i + 1] != '\n'; i++) {
			if (content[i] == ' ') {
				spacesCount++;
				if (spacesCount == 2) {
					string temp = content.substr(start, i - start);
					res.Code = stoi(temp);
				}
				start = i + 1;
			}
		}

		if (res.Code == 404) {
			res.Type = HttpError;
			return res;
		}

		i += 2;
		start = i;

		for (; i < headerDelimPos + 1; i++) {
			if (content[i] == ':' && delim == string::npos) {
				delim = i;
			}
			else if (content[i] == '\r' && content[i + 1] == '\n') {
				if (delim == string::npos) {
					res.Code = -1;
					res.Type = HttpError;
					return res;
				}
				//std::string headerKey = string(content[start], delim - start);
				std::string headerKey;
				std::string temp = headerKey;
				std::transform(temp.begin(), temp.end(), temp.begin(), ::toupper);
				//std::string headerValue = string(content[delim + 2], i - delim - 2);
				std::string headerValue;
				if (temp.find("CONTENT-LENGTH") != std::string::npos)
				{
					res.Headers["CONTENT-LENGTH"] = headerValue;
				}
				else
				{
					res.Headers[headerKey] = headerValue;
				}
				delim = string::npos;
				start = i + 2;
			}
		}
		//res.Content += string(content[i+3], count - i - 3);

		// res.Headers
		THeaders::iterator headIt = res.Headers.find("CONTENT-LENGTH");
		size_t receivedCount = res.Content.length();
		size_t totalCount = receivedCount;
		if (headIt != res.Headers.end())
		{
			totalCount = std::atol(headIt->second.c_str());
		}

		//while (receivedCount < totalCount) {
		//	count = recv(Socket, buffer, 1000, 0);
		//	receivedCount += count;
		//	res.Content += string(buffer, count);
		//}

		if (res.Headers.find("Transfer-Encoding") != res.Headers.end() &&
			res.Headers["Transfer-Encoding"] == "chunked")
		{
			string result;
			size_t i = 0, start = 0;
			for (; i < res.Content.size(); i++) {
				if (res.Content[i] == '\r' && res.Content[i + 1] == '\n') {
					string chunkSizeStr = res.Content.substr(start, i - start);
					size_t chunkSize = stoh(chunkSizeStr);
					result += res.Content.substr(i + 2, chunkSize);
					i += chunkSize + 2;
					start = i + 2;
				}
			}
			res.Content = result;
		}

#ifdef _WIN32
		closesocket(Socket);
		WSACleanup();
#endif
#ifdef __unix__
		shutdown(Socket, SHUT_RDWR);
		close(Socket);
#endif
		res.Type = Success;

		return res;
	}

	inline TAddr ParseUrl(string url) {
		TAddr addr;
		if (url.find("http://") == 0) {
			url = url.substr(7);
		}
		size_t dot = url.find('/');
		if (dot == string::npos) {
			addr.Host = url;
			addr.Action = "/";
			addr.Port = 80;
		}
		else {
			string hostPort = url.substr(0, dot);
			size_t portPos = hostPort.find(':');
			addr.Action = url.substr(dot);
			if (portPos != string::npos) {
				addr.Host = hostPort.substr(0, portPos);
				addr.Port = stoi(hostPort.substr(portPos + 1));
			}
			else {
				addr.Port = 80;
				addr.Host = hostPort;
			}
		}
		return addr;
	}

	inline size_t stoh(const string& str) {
		size_t x;
		std::stringstream ss;
		ss << hex << str;
		ss >> x;
		return x;
	}

}
